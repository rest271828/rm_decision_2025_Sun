#pragma once

#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rm_decision_defs.hpp"
#include "chessboard_interfaces/msg/chessboard.hpp"

namespace RMDecision {

class ChessboardHandle {
public:
    ChessboardHandle(Faction faction) : faction(faction) {
        robots = std::make_shared<std::unordered_map<std::string, std::shared_ptr<Robot>>>();
        terrains = std::make_shared<std::unordered_map<std::string, std::shared_ptr<Terrain>>>();
        architectures = std::make_shared<std::unordered_map<std::string, std::shared_ptr<Architecture>>>();
        initialed = false;
    }

    ChessboardHandle(const chessboard_interfaces::msg::Chessboard& msg);
    // 使用智能指针避免对象切割
    std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Robot>>> robots;
    std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Terrain>>> terrains;
    std::shared_ptr<std::unordered_map<std::string, std::shared_ptr<Architecture>>> architectures;
    RMDecision::Faction faction;
    bool initialed = false;

    inline std::shared_ptr<Robot> friend_robot(const uint& id);

    inline std::shared_ptr<Robot> enemy_robot(const uint& id);

    inline std::shared_ptr<Architecture> friend_outpost();

    inline std::shared_ptr<Architecture> enemy_outpost();

    inline std::shared_ptr<Architecture> friend_base();

    inline std::shared_ptr<Architecture> enemy_base();

    void upgrate_from_message(const chessboard_interfaces::msg::Chessboard& msg);

    chessboard_interfaces::msg::Chessboard to_message();
};

}  // namespace RMDecision