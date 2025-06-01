#pragma once

#include <vector>
#include <string>
#include <map>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "lifecycle_msgs/srv/get_state.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "my_lifecycle_tutorial/supervisor_component.hpp"

namespace lifecycle_supervisor
{

class SupervisorNode : public rclcpp::Node
{
public:
    SupervisorNode();
    ~SupervisorNode() = default;

private:
    // ---- コールバック ----
    void timerCallback();
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void getStateResponse(const std::string& node_name, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future);
    void changeStateResponse(const std::string& node_name, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future);

    // ---- サービスクライアント ----
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedPtr> get_state_clients_;
    std::map<std::string, rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedPtr> change_state_clients_;

    // ---- サブスクライバ ----
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    // ---- 内部状態 ----
    std::vector<std::string> managed_nodes_;          // 管理ノード名（configから取得）
    std::map<std::string, lifecycle_supervisor::Pose2D> node_poses_;   // 各ノードの現在位置
    std::map<std::string, lifecycle_supervisor::Pose2D> node_goals_;   // 各ノードの目標位置
    std::string active_node_;                          // 現在ACTIVEなノード名
    double switch_threshold_;                          // 距離しきい値

    std::map<std::string, int> node_states_;           // 各ノードの状態（enum値等）

    // ---- タイマ ----
    rclcpp::TimerBase::SharedPtr timer_;

    // ---- ロジックAPI ----
    // SupervisorComponent component_;  // 全部static関数化ならインスタンス不要

    // ---- パラメータ等 ----
    // ...必要に応じて追加
};

} // namespace lifecycle_supervisor
