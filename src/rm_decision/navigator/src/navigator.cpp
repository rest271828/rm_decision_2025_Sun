#include "navigator.hpp"

RMDecision::Navigator::Navigator(const rclcpp::NodeOptions &options) : Node("navigator", options){
    nav_msg_sub_ = this->create_subscription<rm_decision_interfaces::msg::Navigate>(
        "to_navigator", 10, std::bind(&Navigator::nav_callback, this, std::placeholders::_1));
    nav_to_pose_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    send_goal_options_.goal_response_callback = std::bind(&Navigator::goal_response_callback, this,
                                                         std::placeholders::_1);
    send_goal_options_.feedback_callback = std::bind(&Navigator::feedback_callback, this, std::placeholders::_1,
                                                    std::placeholders::_2);
    send_goal_options_.result_callback = std::bind(&Navigator::result_callback, this, std::placeholders::_1);

    endtime_ = std::chrono::steady_clock::now();
    nav_state_ = INIT;
    failed_count_ = 0;
    available_ = true;
}

void RMDecision::Navigator::goal_response_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future) {
    auto goal_handle = future;
    if (!goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
        available_ = true;
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
    goal_handle_ = goal_handle;
}

void RMDecision::Navigator::feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback: distance to target point: %.2f m", feedback->distance_remaining);
}

void RMDecision::Navigator::result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was reached!");
        nav_state_ = REACHED;
        available_ = true;
        failed_count_ = 0;
        endtime_ = std::chrono::steady_clock::now();
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(this->get_logger(), "Goal was aborted");
        nav_state_ = ABORTED;
        available_ = true;
        failed_count_++;
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        nav_state_ = CANCELED;
        available_ = true;
        failed_count_++;
        break;
    default:
        RCLCPP_INFO(get_logger(), "Unknown result code");
        nav_state_ = UNKNOWN;
        available_ = true;
        failed_count_++;
        break;
    }
}

void RMDecision::Navigator::nav_callback(const rm_decision_interfaces::msg::Navigate::SharedPtr msg) {
    if (msg->instant) {
        nav_cancel();
    }
    nav_to_pose(msg->pose);
}

void RMDecision::Navigator::nav_to_pose(const geometry_msgs::msg::PoseStamped& msg) {
    nav_to_pose_client_->wait_for_action_server();
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = msg;
    goal_msg.behavior_tree = "";
    available_ = false;
    send_goal_future_ = nav_to_pose_client_->async_send_goal(goal_msg, send_goal_options_);
}

void RMDecision::Navigator::nav_cancel() {
    if (goal_handle_) {
        nav_to_pose_client_->async_cancel_goal(goal_handle_);
        RCLCPP_INFO(this->get_logger(), "Goal cancel request sent");
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(RMDecision::Navigator)