#pragma once

#include <tf2_ros/transform_listener.h>

#include <cassert>
#include <rclcpp/rclcpp.hpp>
#include <type_traits>

#include "information_widgets/rm_msg_includes.hpp"
#include "information_widgets/chessboard_handle.hpp"
#include "information_widgets/prism_handle.hpp"

namespace RMDecision {

class SensingUnit : public rclcpp::Node {
public:
    SensingUnit(const rclcpp::NodeOptions& options);

private:
    ChessboardHandle chessboard_;
    Prism prism_;

    template <typename T1, typename T2>
    void init_map_declare(const std::unordered_map<std::string, T1>& src, std::unordered_map<std::string, std::shared_ptr<T2>>& dist, void (*conversion)(const T1&, T2&) = nullptr) {
        for (const auto& elem : src) {
            const std::string& label = elem.first;
            const T1& defaultData = elem.second;
            this->declare_parameter<T1>(label, defaultData);

            T1 tmpSrcData;
            T2 tmpDistData;
            this->get_parameter<T1>(label, tmpSrcData);

            if (conversion != nullptr) {
                conversion(tmpSrcData, tmpDistData);
            }

            dist[label] = std::make_shared<T2>(tmpDistData);
        }
    }

    template <typename T>
    void name_objects(std::unordered_map<std::string, std::shared_ptr<T>>& map) {
        static_assert(std::is_base_of<Object, T>::value, "T must be a descendant of Object.");
        for (auto& elem : map) {
            elem.second->label = elem.first;
        }
    }

    void init_chessboard(const Faction& faction);
    // chessboard需要的消息订阅
    rclcpp::Subscription<rm_decision_interfaces::msg::AllRobotHP>::SharedPtr all_robot_hp_sub_;
    rclcpp::Subscription<rm_decision_interfaces::msg::FriendLocation>::SharedPtr friend_location_sub_;

    // prism需要的消息订阅
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr tracking_pose_sub_;
    rclcpp::Subscription<rm_decision_interfaces::msg::FromSerial>::SharedPtr from_serial_sub_;
    rclcpp::Subscription<rm_decision_interfaces::msg::ReceiveSerial>::SharedPtr receive_serial_sub_;
    rclcpp::Subscription<rm_decision_interfaces::msg::RobotStatus>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr target_sub_;

    // chessboard发布者
    rclcpp::Publisher<chessboard_interfaces::msg::Chessboard>::SharedPtr chessboard_pub_;

    // prism发布者
    rclcpp::Publisher<rm_decision_interfaces::msg::Prism>::SharedPtr prism_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    void timer_callback();

    void all_robot_hp_callback(const rm_decision_interfaces::msg::AllRobotHP::SharedPtr msg);

    void friend_location_callback(const rm_decision_interfaces::msg::FriendLocation::SharedPtr msg);

    void tracking_pose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);

    void from_serial_callback(const rm_decision_interfaces::msg::FromSerial::SharedPtr msg);

    void receive_serial_callback(const rm_decision_interfaces::msg::ReceiveSerial::SharedPtr msg);

    void robot_status_callback(const rm_decision_interfaces::msg::RobotStatus::SharedPtr msg);

    void target_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg);
};
}  // namespace RMDecision