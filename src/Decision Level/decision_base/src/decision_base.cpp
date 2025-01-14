#include "decision_base/decision_base.hpp"

using namespace RMDecision;

DecisionBase::DecisionBase(uint selfId, std::string nodeName, const rclcpp::NodeOptions &options) 
: Node(nodeName, options), chessboard_(Faction::UNKNOWN) {
    chessboard_sub_ = this->create_subscription<iw_interfaces::msg::Chessboard>(
        "rm_decision/chessboard", 10, 
        std::bind(&DecisionBase::chessboard_sub_callback, this, std::placeholders::_1));
    prism_sub_ = this->create_subscription<iw_interfaces::msg::Prism>(
        ("rm_decision/prism" + std::to_string(selfId)).c_str(), 10,
        std::bind(&DecisionBase::prism_sub_callback, this, std::placeholders::_1));
    nav_pub_ = this->create_publisher<navigator_interfaces::msg::Navigate>("to_navigator", 10);
}

void DecisionBase::chessboard_sub_callback(const iw_interfaces::msg::Chessboard::SharedPtr msg) {
    if (msg->initialed && rclcpp::Time(msg->timestamp) > chessboard_.timestamp) {
        chessboard_.upgrate_from_message(*msg);
    }
}

void DecisionBase::prism_sub_callback(const iw_interfaces::msg::Prism::SharedPtr msg) {
    prism_.update_from_message(*msg);
}

void DecisionBase::nav_to_pose(const PoseStamped& stampedPose, bool instant) {
    navigator_interfaces::msg::Navigate msg;
    msg.pose = stampedPose;
    msg.instant = instant;
    nav_pub_->publish(msg);
}