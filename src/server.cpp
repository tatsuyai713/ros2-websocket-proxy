// ROS 2 WebSocket Proxy — Server Node
//
// Bridges ROS 2 topics over a WebSocket connection.
//  - Subscribes to ROS 2 topics and forwards serialized messages to the
//    connected WebSocket client.
//  - Receives binary frames from the WebSocket client and publishes them
//    to the corresponding ROS 2 topics.
//
// Wire format (binary frame):
//   [ topic_name : TOPIC_HEADER_SIZE bytes (null-padded) ][ CDR payload ]

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <websocketpp/server.hpp>
#include <websocketpp/config/asio_no_tls.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <atomic>
#include <cstring>
#include <mutex>
#include <thread>
#include <unordered_map>

// Fixed-size header that carries the topic name inside every binary frame.
static constexpr size_t TOPIC_HEADER_SIZE = 128;

using WsServer = websocketpp::server<websocketpp::config::asio>;

class GenericServer : public rclcpp::Node
{
public:
    GenericServer()
        : Node("generic_server"), port_(9090), is_connected_(false)
    {
        this->declare_parameter("yaml_file", "server_topics.yaml");
        this->declare_parameter("port", 9090);
        yaml_file_ = this->get_parameter("yaml_file").as_string();
        port_ = this->get_parameter("port").as_int();

        const std::string package_path =
            ament_index_cpp::get_package_share_directory("ros2_websocket_proxy");
        yaml_file_ = package_path + "/config/" + yaml_file_;

        YAML::Node config;
        try {
            config = YAML::LoadFile(yaml_file_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
            rclcpp::shutdown();
            return;
        }

        // ROS → WebSocket: create generic subscribers
        for (const auto &topic : config["subscribe_topics"]) {
            const auto name = topic["name"].as<std::string>();
            const auto type = topic["type"].as<std::string>();
            create_subscriber(name, type);
        }

        // WebSocket → ROS: create generic publishers
        for (const auto &topic : config["publish_topics"]) {
            const auto name = topic["name"].as<std::string>();
            const auto type = topic["type"].as<std::string>();
            publishers_[name] =
                this->create_generic_publisher(name, type, rclcpp::QoS(10));
        }

        // Configure WebSocket server
        server_.init_asio();
        server_.set_open_handler(
            [this](websocketpp::connection_hdl hdl) { on_open(hdl); });
        server_.set_close_handler(
            [this](websocketpp::connection_hdl hdl) { on_close(hdl); });
        server_.set_message_handler(
            [this](websocketpp::connection_hdl hdl, WsServer::message_ptr msg) {
                on_message(hdl, msg);
            });
        server_.clear_access_channels(websocketpp::log::alevel::all);
        server_.clear_error_channels(websocketpp::log::elevel::all);

        server_.listen(port_);
        server_.start_accept();
        server_thread_ = std::thread([this]() { server_.run(); });

        RCLCPP_INFO(this->get_logger(),
                     "WebSocket server listening on port %d", port_);
    }

    ~GenericServer()
    {
        if (server_thread_.joinable()) {
            server_.stop();
            server_thread_.join();
        }
    }

private:
    // Forward a ROS serialized message to the WebSocket client.
    // Single allocation: one vector holds [header | payload] — no intermediate copies.
    void create_subscriber(const std::string &topic_name,
                           const std::string &type_name)
    {
        auto sub = this->create_generic_subscription(
            topic_name, type_name, rclcpp::QoS(10),
            [this, topic_name](std::shared_ptr<rclcpp::SerializedMessage> msg) {
                // Check connection before allocating
                if (!is_connected_.load(std::memory_order_acquire)) {
                    return;
                }

                const auto &rcl_msg = msg->get_rcl_serialized_message();
                const size_t total = TOPIC_HEADER_SIZE + rcl_msg.buffer_length;

                // Build frame in a single buffer
                std::vector<uint8_t> frame(total, 0);
                std::memcpy(frame.data(), topic_name.data(),
                            std::min(topic_name.size(), TOPIC_HEADER_SIZE));
                std::memcpy(frame.data() + TOPIC_HEADER_SIZE,
                            rcl_msg.buffer, rcl_msg.buffer_length);

                std::lock_guard<std::mutex> lock(conn_mutex_);
                if (is_connected_.load(std::memory_order_relaxed)) {
                    try {
                        server_.send(hdl_, frame.data(), total,
                                     websocketpp::frame::opcode::binary);
                    } catch (const websocketpp::exception &e) {
                        RCLCPP_ERROR(this->get_logger(),
                                     "WebSocket send failed: %s", e.what());
                    }
                }
            });
        subscriptions_[topic_name] = sub;
    }

    void on_open(websocketpp::connection_hdl hdl)
    {
        std::lock_guard<std::mutex> lock(conn_mutex_);
        hdl_ = hdl;
        is_connected_.store(true, std::memory_order_release);
        RCLCPP_INFO(this->get_logger(), "WebSocket connection opened.");
    }

    // Decode an incoming binary frame and publish to the matching ROS topic.
    // Works directly on the payload reference — no intermediate vector copies.
    void on_message(websocketpp::connection_hdl /*hdl*/,
                    WsServer::message_ptr msg)
    {
        const std::string &raw = msg->get_payload();
        if (raw.size() < TOPIC_HEADER_SIZE) {
            RCLCPP_ERROR(this->get_logger(),
                         "Payload too small (%zu bytes); need >= %zu.",
                         raw.size(), TOPIC_HEADER_SIZE);
            return;
        }

        // Extract null-terminated topic name from fixed-size header
        const auto *header_end = static_cast<const char *>(
            std::memchr(raw.data(), '\0', TOPIC_HEADER_SIZE));
        const size_t name_len =
            header_end ? static_cast<size_t>(header_end - raw.data())
                       : TOPIC_HEADER_SIZE;
        const std::string topic_name(raw.data(), name_len);

        auto it = publishers_.find(topic_name);
        if (it == publishers_.end()) {
            return;
        }

        // Copy payload directly into SerializedMessage — single memcpy
        const size_t payload_size = raw.size() - TOPIC_HEADER_SIZE;
        rclcpp::SerializedMessage serialized(payload_size);
        auto &rcl_msg = serialized.get_rcl_serialized_message();
        std::memcpy(rcl_msg.buffer,
                    raw.data() + TOPIC_HEADER_SIZE, payload_size);
        rcl_msg.buffer_length = payload_size;

        it->second->publish(serialized);
    }

    void on_close(websocketpp::connection_hdl /*hdl*/)
    {
        std::lock_guard<std::mutex> lock(conn_mutex_);
        is_connected_.store(false, std::memory_order_release);
        RCLCPP_INFO(this->get_logger(), "WebSocket connection closed.");
    }

    // ROS parameters
    std::string yaml_file_;
    int port_;

    // WebSocket
    WsServer server_;
    std::thread server_thread_;
    websocketpp::connection_hdl hdl_;
    std::mutex conn_mutex_;
    std::atomic<bool> is_connected_;

    // ROS pub/sub
    std::unordered_map<std::string, std::shared_ptr<rclcpp::GenericPublisher>>
        publishers_;
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>
        subscriptions_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GenericServer>());
    rclcpp::shutdown();
    return 0;
}
