#include "my_lifecycle_tutorial/supervisor_node.hpp"

namespace lifecycle_supervisor
{

SupervisorNode::SupervisorNode()
: rclcpp::Node("supervisor_node")
{
    // --- パラメータ取得（例） ---
    // declare_parameter("managed_nodes", std::vector<std::string>{});
    // declare_parameter("switch_threshold", 1.0);
    // get_parameter("managed_nodes", managed_nodes_);
    // get_parameter("switch_threshold", switch_threshold_);

    // --- サービスクライアントの作成 ---
    for (const auto& node_name : managed_nodes_) {
        get_state_clients_[node_name] = this->create_client<lifecycle_msgs::srv::GetState>(node_name + "/get_state");
        change_state_clients_[node_name] = this->create_client<lifecycle_msgs::srv::ChangeState>(node_name + "/change_state");
    }

    // --- サブスクライバの作成 ---
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10,
        std::bind(&SupervisorNode::odomCallback, this, std::placeholders::_1)
    );

    // --- タイマの作成 ---
    timer_ = this->create_wall_timer(
        std::chrono::seconds(1),
        std::bind(&SupervisorNode::timerCallback, this)
    );
}

void SupervisorNode::timerCallback()
{
    // 1. 各ノードの状態をget_stateで問い合わせ
    // 2. 最新の状態・位置・目標・閾値などをcomponent::Inputs構造体にまとめる
    // 3. SupervisorComponentの静的関数（判定API）を呼ぶ
    // 4. 判定結果に応じてchange_stateを呼ぶ
    // 5. 必要に応じて状態やログを更新
}

void SupervisorNode::odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // どのノード用かをトピックやframe_idなどで判別し、
    // node_poses_[node_name] = Pose2D{...} で更新
}

void SupervisorNode::getStateResponse(
    const std::string& node_name,
    rclcpp::Client<lifecycle_msgs::srv::GetState>::SharedFuture future)
{
    // future.get()->current_state.id からnode_states_[node_name]を更新
}

void SupervisorNode::changeStateResponse(
    const std::string& node_name,
    rclcpp::Client<lifecycle_msgs::srv::ChangeState>::SharedFuture future)
{
    // future.get()->success などで成否判定し、必要ならログや状態を記録
}

} // namespace lifecycle_supervisor