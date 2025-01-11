#include <rclcpp/rclcpp.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

using namespace std::placeholders;

class NavigateClient : public rclcpp::Node
{
public:
  NavigateClient() : Node("navigate_client")
  {
    client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");
  }

  void send_goal()
  {
    auto goal_msg = nav2_msgs::action::NavigateToPose::Goal();
    // 设置目标消息...

    auto send_goal_options = rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&NavigateClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&NavigateClient::feedback_callback, this, _1);
    send_goal_options.result_callback =
      std::bind(&NavigateClient::result_callback, this, _1);

    future_goal_handle_ = client_->async_send_goal(goal_msg, send_goal_options);
  }

  void cancel_goal()
  {
    if (goal_handle_)
    {
      client_->async_cancel_goal(goal_handle_);
      RCLCPP_INFO(this->get_logger(), "Goal cancel request sent");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "No goal to cancel");
    }
  }

private:
  rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr client_;
  rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr goal_handle_;
  std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> future_goal_handle_;

  void goal_response_callback(std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::GoalResponse> future)
  {
    if (future.get() == rclcpp_action::GoalResponse::ACCEPT)
    {
      RCLCPP_INFO(this->get_logger(), "Goal accepted");
      goal_handle_ = future_goal_handle_.get();
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal rejected");
    }
  }

  void feedback_callback(nav2_msgs::action::NavigateToPose::Feedback::ConstSharedPtr feedback)
  {
    RCLCPP_INFO(this->get_logger(), "Received feedback");
  }

  void result_callback(const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult & result)
  {
    if (result.code == rclcpp_action::ResultCode::SUCCEEDED)
    {
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
    else
    {
      RCLCPP_INFO(this->get_logger(), "Goal failed");
    }
    goal_handle_.reset();
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NavigateClient>();
  node->send_goal();
  rclcpp::spin(node);
  node->cancel_goal(); // 取消目标
  rclcpp::shutdown();
  return 0;
}
