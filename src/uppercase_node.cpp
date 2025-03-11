#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using rclcpp_lifecycle::LifecycleNode;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class UppercaseNode : public LifecycleNode
{
public:
  explicit UppercaseNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : LifecycleNode("uppercase_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "UppercaseNode constructor");
  }

protected:
  CallbackReturn on_configure(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "UppercaseNode: on_configure()");
    publisher_ = this->create_publisher<std_msgs::msg::String>("string_topic", 10);
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "UppercaseNode: on_activate()");
    if (publisher_) {
      publisher_->on_activate();
    }
    timer_ = this->create_wall_timer(1s, [this]() {
      auto msg = std::make_shared<std_msgs::msg::String>();
      msg->data = "ABCDEFG";
      publisher_->publish(*msg);
      RCLCPP_INFO(this->get_logger(), "[UppercaseNode] Publishing: %s", msg->data.c_str());
    });
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "UppercaseNode: on_deactivate()");
    if (timer_) {
      timer_->cancel();
      timer_.reset();
    }
    if (publisher_) {
      publisher_->on_deactivate();
    }
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(this->get_logger(), "UppercaseNode: on_cleanup()");
    publisher_.reset();
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state)
  {
    RCLCPP_INFO(this->get_logger(), "UppercaseNode: on_shutdown() from [%s]", state.label().c_str());
    return CallbackReturn::SUCCESS;
  }

private:
  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UppercaseNode>();
  rclcpp::executors::SingleThreadedExecutor exe;
  exe.add_node(node->get_node_base_interface());
  exe.spin();
  rclcpp::shutdown();
  return 0;
}
