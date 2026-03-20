// ROS 2 WebSocket Proxy — Client Node
//
// Connects to a remote WebSocket server and bridges ROS 2 topics.
//  - Subscribes to local ROS 2 topics and forwards serialized messages
//    to the WebSocket server.
//  - Receives binary frames from the server and publishes them to the
//    corresponding local ROS 2 topics.
//  - Automatically reconnects on connection loss.
//
// Wire format (binary frame):
//   [ topic_name : TOPIC_HEADER_SIZE bytes (null-padded) ][ CDR payload ]

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/generic_publisher.hpp>
#include <rclcpp/generic_subscription.hpp>
#include <websocketpp/client.hpp>
#include <websocketpp/config/asio_no_tls_client.hpp>
#include <yaml-cpp/yaml.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <atomic>
#include <cstring>
#include <mutex>
#include <thread>
#include <unordered_map>

// Must match the server's header size.
static constexpr size_t TOPIC_HEADER_SIZE = 128;

using WsClient = websocketpp::client<websocketpp::config::asio_client>;

class GenericClient : public rclcpp::Node
{
public:
    GenericClient()
        : Node("generic_client"),
          reconnect_interval_(1),
          is_connected_(false)
    {
        this->declare_parameter("yaml_file", "client_topics.yaml");
        this->declare_parameter("ws_url", "ws://localhost:9090");
        yaml_file_ = this->get_parameter("yaml_file").as_string();
        ws_url_ = this->get_parameter("ws_url").as_string();

        const std::string package_path =
            ament_index_cpp::get_package_share_directory("ros2_websocket_proxy");
        yaml_file_ = package_path + "/config/" + yaml_file_;

        YAML::Node config;
        try {
            config = YAML::LoadFile(yaml_file_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(),
                         "Failed to load YAML file: %s", e.what());
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

        init_ws_client();
        client_thread_ = std::thread([this]() { reconnect_loop(); });

        RCLCPP_INFO(this->get_logger(),
                     "Connecting to %s ...", ws_url_.c_str());
    }

    ~GenericClient()
    {
        if (client_thread_.joinable()) {
            client_thread_.join();
        }
    }

private:
    // (Re-)initialize the websocketpp client and register all handlers.
    void init_ws_client()
    {
        ws_client_.init_asio();
        ws_client_.set_open_handler(
            [this](websocketpp::connection_hdl hdl) { on_open(hdl); });
        ws_client_.set_message_handler(
            [this](websocketpp::connection_hdl hdl, WsClient::message_ptr msg) {
                on_message(hdl, msg);
            });
        ws_client_.set_close_handler(
            [this](websocketpp::connection_hdl hdl) { on_close(hdl); });
        ws_client_.set_fail_handler(
            [this](websocketpp::connection_hdl hdl) { on_fail(hdl); });
        ws_client_.clear_access_channels(websocketpp::log::alevel::all);
        ws_client_.clear_error_channels(websocketpp::log::elevel::all);
    }

    // Keep trying to connect while the ROS context is alive.
    void reconnect_loop()
    {
        while (rclcpp::ok()) {
            if (!is_connected_.load(std::memory_order_acquire)) {
                RCLCPP_INFO(this->get_logger(), "Connecting...");
                connect();
            }
            std::this_thread::sleep_for(
                std::chrono::seconds(reconnect_interval_));
        }
    }

    void connect()
    {
        websocketpp::lib::error_code ec;
        auto con = ws_client_.get_connection(ws_url_, ec);
        if (ec) {
            RCLCPP_ERROR(this->get_logger(),
                         "Connection creation failed: %s",
                         ec.message().c_str());
            return;
        }

        {
            std::lock_guard<std::mutex> lock(conn_mutex_);
            client_connection_ = con;
        }

        try {
            ws_client_.connect(con);
            ws_client_.run();
        } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(),
                         "Exception during connect: %s", e.what());
        }
    }

    // Forward a ROS serialized message to the WebSocket server.
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
                if (is_connected_.load(std::memory_order_relaxed) &&
                    client_connection_) {
                    auto send_ec = client_connection_->send(
                        frame.data(), total,
                        websocketpp::frame::opcode::binary);
                    if (send_ec) {
                        RCLCPP_ERROR(this->get_logger(),
                                     "Send failed: %s",
                                     send_ec.message().c_str());
                    }
                }
            });
        subscriptions_[topic_name] = sub;
    }

    void on_open(websocketpp::connection_hdl /*hdl*/)
    {
        is_connected_.store(true, std::memory_order_release);
        RCLCPP_INFO(this->get_logger(), "WebSocket connection opened.");
    }

    // Decode an incoming binary frame and publish to the matching ROS topic.
    // Works directly on the payload reference — no intermediate vector copies.
    void on_message(websocketpp::connection_hdl /*hdl*/,
                    WsClient::message_ptr msg)
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
        is_connected_.store(false, std::memory_order_release);
        RCLCPP_WARN(this->get_logger(),
                     "WebSocket connection closed. Will reconnect...");
        reset_ws_client();
    }

    void on_fail(websocketpp::connection_hdl /*hdl*/)
    {
        is_connected_.store(false, std::memory_order_release);
        RCLCPP_ERROR(this->get_logger(),
                      "WebSocket connection failed. Will reconnect...");
        reset_ws_client();
    }

    // Reset internal ASIO state so a fresh connection can be established.
    void reset_ws_client()
    {
        ws_client_.reset();
        init_ws_client();
    }

    // ROS parameters
    std::string ws_url_;
    std::string yaml_file_;
    int reconnect_interval_;

    // WebSocket
    WsClient ws_client_;
    WsClient::connection_ptr client_connection_;
    std::thread client_thread_;
    std::mutex conn_mutex_;
    std::atomic<bool> is_connected_;

    // ROS pub/sub
    std::unordered_map<std::string, rclcpp::SubscriptionBase::SharedPtr>
        subscriptions_;
    std::unordered_map<std::string, rclcpp::GenericPublisher::SharedPtr>
        publishers_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GenericClient>());
    rclcpp::shutdown();
    return 0;
}
