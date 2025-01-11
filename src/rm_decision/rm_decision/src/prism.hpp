#pragma once

#include <chrono>
#include <future>
#include <string>
#include <unordered_map>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/client_goal_handle.hpp"
#include "rm_decision_defs.hpp"

namespace RMDecision {
class Self {
public:
    uint id;
    uint hp;
    uint ammo;
    uint shooter_heat;
    PoseStamped pose;
};

class Track {
public:
    bool tracking;
    uint id;
    uint hp;
    PoseStamped pose;
};

class Game {
public:
    bool game_start;
    bool buy_ammo_ordered;
    uint coins;
    uint projectile_allowance;
};

class Call {
public:
    PlaneCoordinate target;
    bool is_called;
};

class Prism {
public:
    Prism(){}
    Prism(const rm_decision_interfaces::msg::Prism& msg);

    std::unordered_map<std::string, double> parameters;
    Self self;
    Track track;
    Game game;
    Call call;

    void upgrade_from_message(const rm_decision_interfaces::msg::Prism& msg);

    rm_decision_interfaces::msg::Prism to_message();
};
}  // namespace RMDecision

RMDecision::Prism::Prism(const rm_decision_interfaces::msg::Prism& msg) {
    upgrade_from_message(msg);
}

void RMDecision::Prism::upgrade_from_message(const rm_decision_interfaces::msg::Prism& msg) {
    self.id = msg.self_id;
    self.hp = msg.self_hp;
    self.ammo = msg.self_ammo;
    self.shooter_heat = msg.self_shooter_heat;
    self.pose = msg.self_pose;

    track.tracking = msg.track_tracking;
    track.id = msg.track_id;
    track.hp = msg.track_hp;
    track.pose = msg.track_pose;

    game.game_start = msg.game_game_start;
    game.buy_ammo_ordered = msg.game_buy_ammo_ordered;
    game.coins = msg.game_coins;
    game.projectile_allowance = msg.game_projectile_allowance;

    call.target.x = msg.call_target_x;
    call.target.y = msg.call_target_y;
    call.is_called = msg.call_is_called;
}

rm_decision_interfaces::msg::Prism RMDecision::Prism::to_message() {
    rm_decision_interfaces::msg::Prism msg;
    msg.self_id = self.id;
    msg.self_hp = self.hp;
    msg.self_ammo = self.ammo;
    msg.self_shooter_heat = self.shooter_heat;
    msg.self_pose = self.pose;

    msg.track_tracking = track.tracking;
    msg.track_id = track.id;
    msg.track_hp = track.hp;
    msg.track_pose = track.pose;

    msg.game_game_start = game.game_start;
    msg.game_buy_ammo_ordered = game.buy_ammo_ordered;
    msg.game_coins = game.coins;
    msg.game_projectile_allowance = game.projectile_allowance;

    return msg;
}