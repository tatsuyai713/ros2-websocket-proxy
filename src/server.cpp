#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <fstream>
#include <unordered_map>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <sstream>

static const std::string base64_chars =
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";
class GenericPublisherServer : public rclcpp::Node
{
public:
    GenericPublisherServer() : Node("generic_publisher_server")
    {
        this->declare_parameter("yaml_file", "topics.yaml");
        this->get_parameter("yaml_file", yaml_file_);
        std::string package_path = ament_index_cpp::get_package_share_directory("ros2_websocket_proxy");
        yaml_file_ = package_path + "/config/" + yaml_file_;

        // YAMLファイルを読み込む
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
            topic_names_.insert(topic_name);
            create_publisher(topic_name, type_name);
        }

        server_.init_asio();
        server_.set_open_handler(std::bind(&GenericPublisherServer::on_open, this, std::placeholders::_1));
        server_.set_close_handler(std::bind(&GenericPublisherServer::on_close, this, std::placeholders::_1));
        server_.set_message_handler(std::bind(&GenericPublisherServer::on_message, this, std::placeholders::_1, std::placeholders::_2));
        server_.listen(9090);
        server_.start_accept();
        server_thread_ = std::thread([this]()
                                     { server_.run(); });
    }

    ~GenericPublisherServer()
    {
        if (server_thread_.joinable())
        {
            server_thread_.join();
        }
    }

private:
    std::shared_ptr<rclcpp::GenericPublisher> create_publisher(const std::string &topic_name, const std::string &type_name)
    {
        auto pub = this->create_generic_publisher(topic_name, type_name, rclcpp::QoS(10));
        if (!pub)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create publisher for topic: %s", topic_name.c_str());
            return nullptr;
        }
        publishers_[topic_name] = pub;
        return pub;
    }

    void on_open(websocketpp::connection_hdl hdl)
    {
        RCLCPP_INFO(this->get_logger(), "WebSocket connection opened.");
    }

    void on_close(websocketpp::connection_hdl hdl)
    {
        RCLCPP_INFO(this->get_logger(), "WebSocket connection closed.");
    }

    void on_message(websocketpp::connection_hdl hdl, websocketpp::server<websocketpp::config::asio>::message_ptr msg)
    {
        std::string received_payload = msg->get_payload();
        RCLCPP_INFO(this->get_logger(), "Received message: %s", received_payload.c_str());
        size_t separator_pos = received_payload.find(':');

        if (separator_pos != std::string::npos)
        {
            std::string topic_name = received_payload.substr(0, separator_pos);
            std::string data = received_payload.substr(separator_pos + 1);

            if (topic_names_.find(topic_name) != topic_names_.end())
            {
                auto it = publishers_.find(topic_name);
                if (it != publishers_.end())
                {
                    auto message = rclcpp::SerializedMessage();

                    std::vector<uint8_t> payload = base64_to_binary(data);

                    message.reserve(payload.size());
                    std::memcpy(message.get_rcl_serialized_message().buffer, payload.data(), payload.size());
                    message.get_rcl_serialized_message().buffer_length = payload.size();

                    it->second->publish(message);
                }
            }
        }
    }

    std::vector<uint8_t> base64_to_binary(const std::string &base64)
    {
        std::vector<uint8_t> binary;
        int val = 0;
        int valb = -8;

        for (unsigned char c : base64)
        {
            if (isspace(c) || c == '=')
            {
                continue; // 空白やパディングは無視
            }

            auto pos = base64_chars.find(c);
            if (pos == std::string::npos)
            {
                throw std::invalid_argument("Invalid Base64 string");
            }

            val = (val << 6) + pos;
            valb += 6;

            if (valb >= 0)
            {
                binary.push_back((val >> valb) & 0xFF);
                valb -= 8;
            }
        }

        return binary;
    }

    std::string yaml_file_;
    websocketpp::server<websocketpp::config::asio> server_;
    std::unordered_set<std::string> topic_names_;
    std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>> publishers_;
    std::thread server_thread_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<GenericPublisherServer>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
