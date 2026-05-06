#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class CppHeartbeatNode : public rclcpp::Node
{
public:
    CppHeartbeatNode()
        : Node("cpp_heartbeat_node"), count_(0)
    {
        const std::string topic = this->declare_parameter<std::string>("topic", "/cpp_heartbeat");
        const int period_ms = this->declare_parameter<int>("period_ms", 1000);

        publisher_ = this->create_publisher<std_msgs::msg::String>(topic, 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(period_ms),
            std::bind(&CppHeartbeatNode::on_timer, this));

        RCLCPP_INFO(this->get_logger(), "Publishing heartbeat on %s every %d ms", topic.c_str(), period_ms);
    }

private:
    void on_timer()
    {
        std_msgs::msg::String msg;
        msg.data = "heartbeat #" + std::to_string(count_++);
        publisher_->publish(msg);
    }

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::size_t count_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CppHeartbeatNode>());
    rclcpp::shutdown();
    return 0;
}
