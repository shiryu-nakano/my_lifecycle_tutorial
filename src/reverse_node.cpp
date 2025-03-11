#include <chrono>
#include <memory>
#include <string>
#include <algorithm>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class SakasamaNode : public rclcpp::Node
{
public:
  explicit SakasamaNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("reverse_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "SakasamaNode constructor");

    // サブスクライブ: string_topic
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "string_topic", 10,
      std::bind(&SakasamaNode::callbackString, this, std::placeholders::_1));

    // パブリッシャ: sakasama_topic
    publisher_ = this->create_publisher<std_msgs::msg::String>("sakasama_topic", 10);
  }

private:
  void callbackString(const std_msgs::msg::String::SharedPtr msg)
  {
    auto reversed_str = msg->data;
    std::reverse(reversed_str.begin(), reversed_str.end());

    std_msgs::msg::String out_msg;
    out_msg.data = reversed_str;

    // 受け取ったら即パブリッシュ
    publisher_->publish(out_msg);
    RCLCPP_INFO(this->get_logger(), "[ReverseNode] Reversed Letters: %s", out_msg.data.c_str());
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SakasamaNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
