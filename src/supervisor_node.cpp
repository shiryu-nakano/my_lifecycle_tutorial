// supervisor_node.cpp

#include <chrono>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>
#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;

class Supervisor : public rclcpp::Node
{
public:
  Supervisor()
  : Node("supervisor")
  {
    RCLCPP_INFO(this->get_logger(), "Supervisor constructor");

    // ライフサイクル制御対象ノード (SakasamaNodeは除く)
    target_nodes_ = {
      "lowercase_node",
      "uppercase_node"
    };

    // /ノード名/change_state クライアントを作成
    for (auto & node_name : target_nodes_) {
      auto client = this->create_client<lifecycle_msgs::srv::ChangeState>(
        node_name + "/change_state"
      );
      clients_[node_name] = client;
    }

    start_time_ = now().seconds();

    // 1秒おきに状態を管理
    timer_ = this->create_wall_timer(1s, std::bind(&Supervisor::timerCallback, this));
  }

private:
  void timerCallback()
  {
    double elapsed = now().seconds() - start_time_;

    // 0秒でLowercaseNodeをActivate
    if (!hiragana_activated_ && elapsed >= 0.0) {
      RCLCPP_INFO(this->get_logger(), "Activating LowercaseNode...");
      configure_and_activate("lowercase_node");
      hiragana_activated_ = true;
    }

    // 20秒でLowercaseNodeをDeactivate → UppercaseNodeをActivate
    if (!katakana_activated_ && elapsed >= 20.0) {
      RCLCPP_INFO(this->get_logger(), "Switching to UppercaseNode...");
      deactivate("lowercase_node");
      configure_and_activate("uppercase_node");
      katakana_activated_ = true;
    }

    // 40秒で全LifecycleノードをShutdown
    if (!done_ && elapsed >= 40.0) {
      RCLCPP_INFO(this->get_logger(), "Shutting down all lifecycle nodes...");
      shutdown_lifecycle_node("lowercase_node");
      shutdown_lifecycle_node("uppercase_node");
      done_ = true;

      // Supervisor自体も終了
      rclcpp::shutdown();
    }
  }

  // Configure → Activate
  void configure_and_activate(const std::string & node_name)
  {
    change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);
    change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE);
  }

  // Deactivate
  void deactivate(const std::string & node_name)
  {
    change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
  }

  // Deactivate → Cleanup → Shutdown
  void shutdown_lifecycle_node(const std::string & node_name)
  {
    change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_DEACTIVATE);
    change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_CLEANUP);
    change_state(node_name, lifecycle_msgs::msg::Transition::TRANSITION_UNCONFIGURED_SHUTDOWN);
  }

  /**
   * ライフサイクルノードの /change_state サービスを呼んで状態を変更
   */
  bool change_state(const std::string & node_name, uint8_t transition_id)
{
  auto it = clients_.find(node_name);
  if (it == clients_.end()) {
    RCLCPP_ERROR(this->get_logger(), "No client for node: %s", node_name.c_str());
    return false;
  }
  auto client = it->second;

  // サービスが起動するまで待機 (これはOK。spin不要)
  if (!client->wait_for_service(std::chrono::seconds(2))) {
    RCLCPP_ERROR(this->get_logger(), "Service not available for node: %s", node_name.c_str());
    return false;
  }

  // リクエスト作成
  auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
  request->transition.id = transition_id;

  // 非同期でサービス呼び出し
  auto future = client->async_send_request(
    request,
    [this, node_name](rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture response) {
      // ここは別スレッドで呼ばれるコールバック
      if (!response.get()->success) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to change state for node: %s", node_name.c_str());
      } else {
        RCLCPP_INFO(this->get_logger(),
                    "Succeeded to change state for node: %s", node_name.c_str());
      }
    }
  );

  // ここでは同期待ちせず、即returnする
  // 成否はコールバック内でログを出すだけにする
  return true;
}




  std::vector<std::string> target_nodes_;
  std::unordered_map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> clients_;
  rclcpp::TimerBase::SharedPtr timer_;
  double start_time_;

  bool hiragana_activated_{false};
  bool katakana_activated_{false};
  bool done_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto supervisor = std::make_shared<Supervisor>();
  rclcpp::spin(supervisor);
  rclcpp::shutdown();
  return 0;
}
