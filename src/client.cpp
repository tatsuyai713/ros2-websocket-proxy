#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <thread>
#include <string>
#include <sstream>
#include <iomanip>
#include <ament_index_cpp/get_package_share_directory.hpp>

static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

class GenericSubscriberClient : public rclcpp::Node
{
public:
    GenericSubscriberClient()
        : Node("generic_subscriber_client"),
          reconnect_interval_(1),
          is_connected_(false)
    {
        this->declare_parameter("yaml_file", "topics.yaml");
        this->get_parameter("yaml_file", yaml_file_);
        this->declare_parameter("ws_url", "ws://localhost:9090");
        this->get_parameter("ws_url", ws_url_);
        std::string package_path = ament_index_cpp::get_package_share_directory("ros2_websocket_proxy");
        yaml_file_ = package_path + "/config/" + yaml_file_;

        YAML::Node config;
        try
        {
            config = YAML::LoadFile(yaml_file_);
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        auto topics = config["topics"];
        for (const auto &topic : topics)
        {
            std::string topic_name = topic["name"].as<std::string>();
            std::string type_name = topic["type"].as<std::string>();
            create_subscriber(topic_name, type_name);
        }

        ws_client_.init_asio();
        ws_client_.set_open_handler(std::bind(&GenericSubscriberClient::on_open, this, std::placeholders::_1));
        ws_client_.set_message_handler(std::bind(&GenericSubscriberClient::on_message, this, std::placeholders::_1, std::placeholders::_2));
        ws_client_.set_close_handler(std::bind(&GenericSubscriberClient::on_close, this, std::placeholders::_1));
        ws_client_.set_fail_handler(std::bind(&GenericSubscriberClient::on_fail, this, std::placeholders::_1));

        client_thread_ = std::thread([this]()
                                     { run(); });
    }

    ~GenericSubscriberClient()
    {
        if (client_thread_.joinable())
        {
            client_thread_.join();
        }
    }

private:
    void run()
    {
        while (rclcpp::ok())
        {
            if (!is_connected_)
            {
                connect();
                RCLCPP_INFO(this->get_logger(), "Reconnecting...");
            }
            std::this_thread::sleep_for(std::chrono::seconds(reconnect_interval_));
        }
    }

    void connect()
    {
        websocketpp::lib::error_code ec;
        client_connection_ = ws_client_.get_connection(ws_url_, ec);
        if (ec)
        {
            RCLCPP_ERROR(this->get_logger(), "Could not create connection: %s", ec.message().c_str());
            is_connected_ = false;
            return;
        }

        try
        {
            ws_client_.connect(client_connection_);
            ws_client_.run();
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception while connecting: %s", e.what());
        }
    }

    void create_subscriber(const std::string &topic_name, const std::string &type_name)
    {
        auto sub = this->create_generic_subscription(topic_name, type_name, rclcpp::QoS(10),
                                                     [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg)
                                                     {
                                                         const auto &serialized_msg = msg->get_rcl_serialized_message();
                                                         std::vector<uint8_t> payload(reinterpret_cast<const uint8_t *>(serialized_msg.buffer),
                                                                                      reinterpret_cast<const uint8_t *>(serialized_msg.buffer) + serialized_msg.buffer_length);
                                                         if (is_connected_)
                                                         {
                                                             std::string base64_string = to_base64(payload);

                                                             std::string message_with_topic = topic_name + ":" + base64_string;

                                                             RCLCPP_INFO(this->get_logger(), "Received message: %s", message_with_topic.c_str());

                                                             auto send_ec = client_connection_->send(message_with_topic, websocketpp::frame::opcode::text);

                                                             if (send_ec)
                                                             {
                                                                 RCLCPP_ERROR(this->get_logger(), "Send failed: %s", send_ec.message().c_str());
                                                             }
                                                             else
                                                             {
                                                                 RCLCPP_INFO(this->get_logger(), "Hex string sent successfully: %s", base64_string.c_str());
                                                             }
                                                         }
                                                     });
        subscriptions_[topic_name] = sub;
    }

    std::string to_base64(const std::vector<uint8_t> &data)
    {
        std::string result;
        int val = 0;
        int valb = -6;

        for (uint8_t c : data)
        {
            val = (val << 8) + c;
            valb += 8;
            while (valb >= 0)
            {
                result.push_back(base64_chars[(val >> valb) & 0x3F]);
                valb -= 6;
            }
        }

        if (valb > -6)
        {
            result.push_back(base64_chars[((val << 8) >> valb) & 0x3F]);
        }

        while (result.size() % 4)
        {
            result.push_back('=');
        }

        return result;
    }

    void on_open(websocketpp::connection_hdl hdl)
    {
        RCLCPP_INFO(this->get_logger(), "WebSocket connection opened.");
        is_connected_ = true;
    }

    void on_message(websocketpp::connection_hdl hdl, websocketpp::client<websocketpp::config::asio_client>::message_ptr msg)
    {
        // RCLCPP_INFO(this->get_logger(), "Received message: %s", msg->get_payload().c_str());
    }

    void on_close(websocketpp::connection_hdl hdl)
    {
        RCLCPP_WARN(this->get_logger(), "WebSocket connection closed.");
        is_connected_ = false;
        ws_client_.reset();
        ws_client_.init_asio();
        ws_client_.set_open_handler(std::bind(&GenericSubscriberClient::on_open, this, std::placeholders::_1));
        ws_client_.set_message_handler(std::bind(&GenericSubscriberClient::on_message, this, std::placeholders::_1, std::placeholders::_2));
        ws_client_.set_close_handler(std::bind(&GenericSubscriberClient::on_close, this, std::placeholders::_1));
        ws_client_.set_fail_handler(std::bind(&GenericSubscriberClient::on_fail, this, std::placeholders::_1));
    }

    void on_fail(websocketpp::connection_hdl hdl)
    {
        RCLCPP_ERROR(this->get_logger(), "WebSocket connection failed.");
        is_connected_ = false;
        ws_client_.reset();
        ws_client_.init_asio();
        ws_client_.set_open_handler(std::bind(&GenericSubscriberClient::on_open, this, std::placeholders::_1));
        ws_client_.set_message_handler(std::bind(&GenericSubscriberClient::on_message, this, std::placeholders::_1, std::placeholders::_2));
        ws_client_.set_close_handler(std::bind(&GenericSubscriberClient::on_close, this, std::placeholders::_1));
        ws_client_.set_fail_handler(std::bind(&GenericSubscriberClient::on_fail, this, std::placeholders::_1));
    }

    std::string ws_url_;
    std::string yaml_file_;
    websocketpp::client<websocketpp::config::asio_client> ws_client_;
    websocketpp::client<websocketpp::config::asio_client>::connection_ptr client_connection_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    std::thread client_thread_;
    int reconnect_interval_;
    bool is_connected_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GenericSubscriberClient>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
