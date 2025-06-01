#pragma once

#include <vector>
#include <map>
#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_supervisor/supervisor_component.hpp"

namespace lifecycle_supervisor
{

class SupervisorNode : public rclcpp::Node
{
public:
    SupervisorNode();
    ~SupervisorNode() = default;

private:
    // --- コールバック ---
    void timerCallback();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    // 他にもセンサ購読コールバックがあればここに追加

    // --- Service Clients ---
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> get_state_clients_;
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_state_clients_;

    // --- サブスクライバ ---
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    // 他にも必要なサブスクライバはここに追加

    // --- 内部状態 ---
    std::vector<std::string> managed_nodes_;           // configで与えられる
    std::map<std::string, ManagedState> node_states_;  // 各ノードの現状態
    std::map<std::string, Pose2D> node_poses_;         // 各ノードの最新位置
    std::map<std::string, Pose2D> node_goals_;         // 各ノードのゴール
    std::string active_node_;                          // 現在ACTIVEなノード名
    double switch_threshold_;

    // --- Timer ---
    rclcpp::TimerBase::SharedPtr timer_;

    // --- ロジック ---
    // SupervisorComponent component_; // 必要に応じて
};

} // namespace lifecycle_supervisor
