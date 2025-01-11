#include "geometry_msgs/msg/pose.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rm_decision_interfaces/msg/navigate.hpp"


namespace RMDecision{
enum NavState {
    INIT,
    REACHED,
    ABORTED,
    CANCELED,
    UNKNOWN
};

class Navigator : public rclcpp::Node {
public:
    explicit Navigator(const rclcpp::NodeOptions &options);

private:
    rclcpp::Subscription<rm_decision_interfaces::msg::Navigate>::SharedPtr nav_msg_sub_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client_;
    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options_;
    rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
     std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> send_goal_future_;
    std::chrono::steady_clock::time_point endtime_;
    NavState nav_state_;
    int failed_count_;
    bool available_;

    void nav_callback(const rm_decision_interfaces::msg::Navigate::SharedPtr msg);

    void goal_response_callback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future);

    void feedback_callback(
        rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
        const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback);

    void result_callback(
        const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult& result);

    void nav_to_pose(const geometry_msgs::msg::PoseStamped& msg);

    void nav_cancel();
};
} // namespace RMDecision