#pragma once

// 使用该宏通过自动注册一个节点
#define REGIST_DECISION_NODE(className) \
    int main(int argc, char **argv) { \
        rclcpp::init(argc, argv); \
        auto exec = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(); \
        auto node = std::make_shared<className>(rclcpp::NodeOptions()); \
        exec->add_node(node); \
        exec->spin(); \
        rclcpp::shutdown(); \
        return 0; \
    }
    