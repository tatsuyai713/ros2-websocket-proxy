#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sstream>
#include <cstring>

class GenericServer : public rclcpp::Node
{
public:
    GenericServer() : Node("generic_server"), port_(9090)
    {
        this->declare_parameter("yaml_file", "server_topics.yaml");
        this->get_parameter("yaml_file", yaml_file_);
        this->declare_parameter("port", 9090);
        this->get_parameter("port", port_);
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

        auto subscribe_topics = config["subscribe_topics"];
        for (const auto &topic : subscribe_topics)
        {
            std::string topic_name = topic["name"].as<std::string>();
            std::string type_name = topic["type"].as<std::string>();
            create_subscriber(topic_name, type_name);
        }

        auto publish_topics = config["publish_topics"];
        for (const auto &topic : publish_topics)
        {
            std::string topic_name = topic["name"].as<std::string>();
            std::string type_name = topic["type"].as<std::string>();

            auto publisher = this->create_generic_publisher(topic_name, type_name, rclcpp::QoS(10));
            publishers_[topic_name] = publisher;
        }

        server_.init_asio();
        server_.set_open_handler(std::bind(&GenericServer::on_open, this, std::placeholders::_1));
        server_.set_close_handler(std::bind(&GenericServer::on_close, this, std::placeholders::_1));
        server_.set_message_handler(std::bind(&GenericServer::on_message, this, std::placeholders::_1, std::placeholders::_2));
        
        server_.clear_access_channels(websocketpp::log::alevel::all);
        server_.clear_error_channels(websocketpp::log::elevel::all);

        server_.listen(port_);
        server_.start_accept();
        server_thread_ = std::thread([this]()
                                     { server_.run(); });
    }

    ~GenericServer()
    {
        if (server_thread_.joinable())
        {
            server_.stop();
            server_thread_.join();
        }
    }

private:
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
                                                             std::vector<uint8_t> message_with_topic(128, 0);
                                                             std::copy(topic_name.begin(), topic_name.end(), message_with_topic.begin());
                                                             message_with_topic.insert(message_with_topic.end(), payload.begin(), payload.end());

                                                             server_.send(hdl_, message_with_topic.data(), message_with_topic.size(), websocketpp::frame::opcode::binary);
                                                         }
                                                     });
        subscriptions_[topic_name] = sub;
    }

    void on_open(websocketpp::connection_hdl hdl)
    {
        RCLCPP_INFO(this->get_logger(), "WebSocket connection opened.");
        is_connected_ = true;
        hdl_ = hdl;
    }

    void on_message(websocketpp::connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg)
    {
        std::vector<uint8_t> received_payload(msg->get_payload().begin(), msg->get_payload().end());

        if (received_payload.size() < 128)
        {
            RCLCPP_ERROR(this->get_logger(), "Received payload is too small to contain topic information.");
            return;
        }

        std::string topic_name(received_payload.begin(), received_payload.begin() + 128);
        topic_name.erase(std::find(topic_name.begin(), topic_name.end(), '\0'), topic_name.end());

        auto it = publishers_.find(topic_name);
        if (it != publishers_.end())
        {
            auto message = rclcpp::SerializedMessage();

            std::vector<uint8_t> payload(received_payload.begin() + 128, received_payload.end());
            message.reserve(payload.size());
            std::memcpy(message.get_rcl_serialized_message().buffer, payload.data(), payload.size());
            message.get_rcl_serialized_message().buffer_length = payload.size();

            it->second->publish(message);
        }
    }

    void on_close(websocketpp::connection_hdl hdl)
    {
        RCLCPP_INFO(this->get_logger(), "WebSocket connection closed.");
        is_connected_ = false;
        hdl_ = hdl;
    }

    std::string yaml_file_;
    int port_;
    websocketpp::server<websocketpp::config::asio> server_;
    std::unordered_set<std::string> topic_names_;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> publishers_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
    std::unordered_set<websocketpp::connection_hdl, std::owner_less<websocketpp::connection_hdl>> connections_;
    std::thread server_thread_;
    websocketpp::connection_hdl hdl_;
    bool is_connected_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GenericServer>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
