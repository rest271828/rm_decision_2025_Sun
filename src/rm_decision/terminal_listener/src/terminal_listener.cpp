#include <future>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rm_decision_interfaces/msg/navigate.hpp"

class TerminalListener : public rclcpp::Node {
public:
    TerminalListener() : Node("terminal_listener") {
        nav_pub_ = this->create_publisher<rm_decision_interfaces::msg::Navigate>("to_navigator", 10);
    }

    void set_input_param(const std::string& input) {
        std::istringstream iss(input);
        std::vector<float> values;
        float num;
        while (iss >> num) {
            values.push_back(num);
        }

        if (values.size() == 2) {
            geometry_msgs::msg::PoseStamped msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = "map";
            msg.pose.position.x = values[0];
            msg.pose.position.y = values[1];
            msg.pose.position.z = 0.0;
            msg.pose.orientation.x = 0.0;
            msg.pose.orientation.y = 0.0;
            msg.pose.orientation.z = 0.0;
            msg.pose.orientation.w = 1.0;
            RCLCPP_INFO(
                this->get_logger(), "Publishing: position(%.3f, %.3f)", msg.pose.position.x, msg.pose.position.y);
            rm_decision_interfaces::msg::Navigate nav_msg;
            nav_msg.instant = true;
            nav_msg.pose = msg;
            nav_pub_->publish(nav_msg);
        } else {
            RCLCPP_WARN(this->get_logger(), "Invalid input: excepted 2 arguments.");
        }
    }

private:
    rclcpp::Publisher<rm_decision_interfaces::msg::Navigate>::SharedPtr nav_pub_;
};

void read_input(rclcpp::Node::SharedPtr node) {
    auto terminal_listener = std::dynamic_pointer_cast<TerminalListener>(node);
    std::string input;
    while (rclcpp::ok()) {
        std::getline(std::cin, input);
        if (input == "quit") {
            RCLCPP_INFO(terminal_listener->get_logger(), "Parameter listening stopped, enter < Ctrl+C > to shutdown.");
            break;
        }
        if (!input.empty() && terminal_listener) {
            terminal_listener->set_input_param(input);
        }
    }
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TerminalListener>();
    auto input_future = std::async(std::launch::async, read_input, node);
    rclcpp::spin(node);
    input_future.wait();
    rclcpp::shutdown();
    return 0;
}