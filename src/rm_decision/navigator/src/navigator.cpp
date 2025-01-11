#include "navigator.hpp"

Navigator::Navigator() {
    nav_to_pose_client = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
    send_goal_options.goal_response_callback = std::bind(&Commander::goal_response_callback, this,
                                                         std::placeholders::_1);
    send_goal_options.feedback_callback = std::bind(&Commander::feedback_callback, this, std::placeholders::_1,
                                                    std::placeholders::_2);
    send_goal_options.result_callback = std::bind(&Commander::result_callback, this, std::placeholders::_1);

    nav_state_ = INIT;
    failed_count_ = 0;
    available_ = true;
}

void Navigator::goal_response_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal was rejected by server");
        available = true;
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
    goal_handle_ = goal_handle;
}

void Navigator::feedback_callback(
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
    const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback) {
    RCLCPP_INFO(this->get_logger(), "Received feedback: distance to target point: %.2f m", feedback->distance_remaining);
}

void Navigator::result_callback(
    const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result) {
    switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "Goal was reached!");
        nav_state = REACHED;
        available = true;
        failed_count = 0;
        endtime = std::chrono::steady_clock::now();
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_INFO(this->get_logger(), "Goal was aborted");
        nav_state = ABORTED;
        available = true;
        failed_count++;
        break;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_INFO(this->get_logger(), "Goal was canceled");
        nav_state = CANCELED;
        available = true;
        failed_count++;
        break;
    default:
        RCLCPP_INFO(get_logger(), "Unknown result code");
        nav_state = UNKNOWN;
        available = true;
        failed_count++;
        break;
    }
}

void Navigator::nav(const rm_decision_interfaces::msg::navigate& msg) {
    if (msg.instant) {
        nav_cancel();
    }
    nav_to_pose(msg.pose);
}

void Navigator::nav_to_pose(const geometry_msgs::msg::PoseStamped& msg) {
    nav_to_pose_client->wait_for_action_server();
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    goal_msg.pose = msg;
    goal_msg.behavior_tree = "";
    available_ = false;
    send_goal_future = nav_to_pose_client->async_send_goal(goal_msg, send_goal_options);
}

void nav_cancel() {
    if (goal_handle_) {
        client_->async_cancel_goal(goal_handle_);
        RCLCPP_INFO(this->get_logger(), "Goal cancel request sent");
    }
}