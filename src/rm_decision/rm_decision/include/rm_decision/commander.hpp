//作出决策的节点
#ifndef RM_DECISION__COMMANDER_HPP_
#define RM_DECISION__COMMANDER_HPP_

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "action_msgs/msg/goal_status_array.hpp"

#include <behaviortree_cpp/basic_types.h>
#include <geometry_msgs/msg/detail/pose_stamped__struct.hpp>
#include <memory>
#include <iostream>
#include <nav_msgs/msg/detail/odometry__struct.hpp>
#include <rclcpp/node.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <vector>
#include <thread>
#include <optional>
#include <fstream>
#include <string>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>

#include "rm_decision_interfaces/msg/all_robot_hp.hpp"
#include "auto_aim_interfaces/msg/target.hpp"
#include "rm_decision_interfaces/msg/to_serial.hpp"
#include "rm_decision_interfaces/msg/from_serial.hpp"
#include "rm_decision_interfaces/msg/to_auto_aim.hpp"

#include <sensor_msgs/msg/laser_scan.hpp>
//behave tree
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/plugins.hpp"
#include "behaviortree_ros2/bt_topic_sub_node.hpp"
#include "behaviortree_cpp/loggers/groot2_publisher.h"


namespace rm_decision {


    class Commander;


    class State {
    private:
    public:
        State(Commander *commander) : commander(commander) {}

        virtual ~State() = default;

        virtual void handle() {};
        Commander *commander;
        // virtual std::optional<std::shared_ptr<State>> check() = 0;
    };

    class Robot {
    public:
        Robot() = default;

        Robot(int id, geometry_msgs::msg::PoseStamped pose, uint hp) : id(id), pose(pose), hp(hp) {}

        int id;
        geometry_msgs::msg::PoseStamped pose;
        uint hp;
        bool attack = false;

        void check() {
            if (pose.pose.position.x >= 4.5) {
                attack = true;
            }
        }
    };

    class PatrolState : public State {
    public:
        explicit PatrolState(Commander *commander) : State(commander) {}

        void handle() override;
    };

    class GoAndStayState : public State {
    public:
        explicit GoAndStayState(Commander *commander) : State(commander) {}

        void handle() override;
    };

    class AttackState : public State {
    public:
        explicit AttackState(Commander *commander) : State(commander) {}

        virtual void handle() override;
    };

    class WaitState : public State {
    public:
        explicit WaitState(Commander *commander) : State(commander) {}

        virtual void handle() override;
    };

    class MoveState : public State {
    public:
        explicit MoveState(Commander *commander) : State(commander) {}

        virtual void handle() override;
    };

    class CjState : public State {
    public:
        explicit CjState(Commander *commander) : State(commander) {}

        virtual void handle() override;
    };


    class Commander : public rclcpp::Node {
    public:
        explicit Commander(const rclcpp::NodeOptions &options); // 重载构造函数

        ~Commander() override; // 析构函数

        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SendGoalOptions send_goal_options;

        void nav_to_pose(geometry_msgs::msg::PoseStamped goal_pose);

        void
        goal_response_callback(rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future);

        void feedback_callback(
                rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr future,
                const std::shared_ptr<const nav2_msgs::action::NavigateToPose::Feedback> feedback
        );

        void result_callback(
                const rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::WrappedResult &result);

        double distence(geometry_msgs::msg::PoseStamped a);

        void processPoses(std::vector<double> &pose_param, std::vector<geometry_msgs::msg::PoseStamped> &nav_points_);

        void getcurrentpose();

        void canshangpo();

        bool isinpo(std::vector<geometry_msgs::msg::PoseStamped> area, geometry_msgs::msg::PoseStamped goal);

        float getyawdiff(geometry_msgs::msg::PoseStamped a, geometry_msgs::msg::PoseStamped b);

        std::vector<geometry_msgs::msg::PoseStamped> generateRandomPoints(int num_points, double radius);

        geometry_msgs::msg::PoseStamped currentpose;
        // 导航点
        std::vector<geometry_msgs::msg::PoseStamped> Guard_points;
        std::vector<geometry_msgs::msg::PoseStamped> Guard_points2;

        std::vector<geometry_msgs::msg::PoseStamped> self_addhp_point;
        std::vector<geometry_msgs::msg::PoseStamped> self_base_point;
        std::vector<geometry_msgs::msg::PoseStamped> S1_Stop_Engineer_point;
        std::vector<geometry_msgs::msg::PoseStamped> S1_Stop_Hero_points;
        std::vector<geometry_msgs::msg::PoseStamped> S1_Outpost_point;
        std::vector<geometry_msgs::msg::PoseStamped> S2_Outpose_point;
        std::vector<geometry_msgs::msg::PoseStamped> S3_Patro_points;

        std::vector<geometry_msgs::msg::PoseStamped> Patro_points;


        std::vector<geometry_msgs::msg::PoseStamped> move_points_;
        std::vector<geometry_msgs::msg::PoseStamped> po_area1;
        std::vector<geometry_msgs::msg::PoseStamped> po_area2;
        std::vector<geometry_msgs::msg::PoseStamped> po_area3;
        std::vector<std::vector<geometry_msgs::msg::PoseStamped>> po_name = {po_area1, po_area2, po_area3};


        std::vector<std::vector<geometry_msgs::msg::PoseStamped>> list_name = {Guard_points, self_addhp_point,
                                                                               self_base_point, S1_Stop_Engineer_point,
                                                                               S1_Stop_Hero_points, S1_Outpost_point,
                                                                               S2_Outpose_point, S3_Patro_points,
                                                                               Guard_points2
                                                                               };
        std::vector<geometry_msgs::msg::PoseStamped>::iterator random;
        std::vector<geometry_msgs::msg::PoseStamped>::iterator attack;

        std::vector<geometry_msgs::msg::PoseStamped>::iterator move;
        geometry_msgs::msg::PoseStamped goal;
        geometry_msgs::msg::PoseStamped home;
        std::shared_future<rclcpp_action::ClientGoalHandle<nav2_msgs::action::NavigateToPose>::SharedPtr> send_goal_future;
        float diffyaw = 0.0 ;
        bool checkpo_shangpoing = false;
        bool checkgoal = true;
        bool first_start = false;
        bool control = false;
        std::chrono::steady_clock::time_point startTime;



        // 机器人血量
        Robot self_1 = Robot(1, geometry_msgs::msg::PoseStamped(), 150);
        Robot self_2 = Robot(2, geometry_msgs::msg::PoseStamped(), 150);
        Robot self_3 = Robot(3, geometry_msgs::msg::PoseStamped(), 150);
        Robot self_4 = Robot(4, geometry_msgs::msg::PoseStamped(), 150);
        Robot self_5 = Robot(5, geometry_msgs::msg::PoseStamped(), 150);
        Robot self_7 = Robot(7, geometry_msgs::msg::PoseStamped(), 600);
        Robot enemy_1 = Robot(1, geometry_msgs::msg::PoseStamped(), 150);
        Robot enemy_2 = Robot(2, geometry_msgs::msg::PoseStamped(), 150);
        Robot enemy_3 = Robot(3, geometry_msgs::msg::PoseStamped(), 150);
        Robot enemy_4 = Robot(4, geometry_msgs::msg::PoseStamped(), 150);
        Robot enemy_5 = Robot(5, geometry_msgs::msg::PoseStamped(), 150);
        Robot enemy_7 = Robot(7, geometry_msgs::msg::PoseStamped(), 600);

        uint enemy_outpost_hp;
        uint enemy_base_hp = 1500;

        int enemy_num = 0;

        uint enemyhp();

        double start;
        uint event_data;

        // 敌方机器人坐标
        geometry_msgs::msg::PoseStamped enemypose;
        bool tracking = false;
        bool buxue = false;


        //for bt
        bool buy_ammo_ordered = false;
        int color; //1 red 2 blue
        bool gamestart = false;
        float self_hp = 400;
        float self_ammo = 400;
        float self_base = 400;
        float self_outpost = 1500;
        int nav_state;  //1 for SUCCEEDED 2 for ABORTED 3 for CANCELED 4 for RUNNING
        float goldcoin;
        bool defend_order_goal_reached = false;
        int strategy;
        bool addhp_ordered;
        int buy_ammo_num=0;
        int buy_ammo_remotely_times=0;
        int remote_buy_ammo_times=0;
        int buy_hp_times=0;
        bool ammoBought = false;
        bool ammoBoughtremotely = false;
        bool hpBought = false;
        bool stop_engineer_goal_reached = false;
        bool stop_engineer_finished = false; //will not stop in s1 if set to true
        bool shangpofail = true;
        bool shanpotimer = false;
        bool addhpfail = false;
        bool addhptimer = false;
        int sentry_status;
        std::chrono::steady_clock::time_point start_time;
        std::chrono::steady_clock::time_point start_time2;

        //一些参数文件
        int not_buy_but_relive = 1;
        int sentry_want_shangpo =2;
        int addhp_threshold = 100; //加血阈值，低于这个血量就会去加血
        int addhp_full_threshold = 400; //加血满阈值，防止哨兵到了加血点就回去
        int defend_threshold = 4000; //当基地血量少于此值，就会触发防御模式
        int buy_to_relive_goldcoin_threshold = 400; //当金币数大于此值，会请求花金币复活

        //远程买弹的条件有两种 1、无敌且有钱 2、不无敌，但血多且有钱 下面的参数需要同时满足多个才会请求远程买弹
        int buy_ammo_remotely_ammo_threshold_when_wudi = 50; //无敌时，子弹数小于此值
        int buy_ammo_remotely_ammo_threshold_when_youdi = 50; //不无敌时，子弹数小于此值
        int buy_ammo_remotely_hp_threshold_when_youdi = 50; //不无敌时，自身血量大于此值
        int buy_ammo_remotely_goldcoin_threshold_when_wudi = 400; //无敌时，当金币数大于此值
        int buy_ammo_remotely_goldcoin_threshold_when_youdi = 400; //不无敌时，当金币数大于此值

        //花钱远程补血的条件 血少且钱多
        int buy_hp_remotely_hp_threshold = 200;
        int buy_hp_remotely_goldcoin_threshold = 300;

        //回基地补弹条件 没弹有点钱
        int buy_ammo_local_threshold = 50;
        int buy_ammo_local_goldcoin_threshold = 200;

        int self_outpose_threshold = 500; //当己方前哨站血量少于此值，会从出击策略转为防御策略
        int buy_ammo_num_at_a_time = 100; //哨兵每一次买弹量

        int time_to_stop_enginner = 5; //S1决策中，哨兵停留在工程点的时间
    private:

        rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr nav_to_pose_client;

        void decision();

        void executor();

        void setState(std::shared_ptr<State> state);

        void loadNavPoints();
        void checkpo();

        void aim_callback(const auto_aim_interfaces::msg::Target::SharedPtr msg);

        void serial_callback(const rm_decision_interfaces::msg::FromSerial::SharedPtr msg);

        void enemypose_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
        // void cmd;


        std::shared_ptr<State> currentState;

        rclcpp::Subscription<rm_decision_interfaces::msg::FromSerial>::SharedPtr nav_sub_;
        rclcpp::Subscription<auto_aim_interfaces::msg::Target>::SharedPtr aim_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr enemypose_sub_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

        rclcpp::Publisher<rm_decision_interfaces::msg::ToSerial>::SharedPtr sentry_cmd_pub_;
        rclcpp::Publisher<rm_decision_interfaces::msg::ToAutoAim>::SharedPtr sentry_status_pub_;


        std::thread commander_thread_;
        std::thread executor_thread_;

        std::shared_ptr<tf2_ros::Buffer> tf2_buffer_;
        std::shared_ptr<tf2_ros::TransformListener> tf2_listener_;
        // tf2_ros::Buffer buffer{get_clock()};
        // tf2_ros::TransformListener tflistener{buffer};

        //this is used for bt

        void mygimbal_handle() {
            std::cout << "mygimbal_handle is called" << std::endl;
        }
        void myGoToEnemyOutpose_handle() {
            goal.header.stamp = this->now();
            goal.header.frame_id = "map";
            goal.pose.position.x = S1_Outpost_point[0].pose.position.x;
            goal.pose.position.y = S1_Outpost_point[0].pose.position.y;
            goal.pose.position.z = S1_Outpost_point[0].pose.position.z;
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;
            setState(std::make_shared<GoAndStayState>(this));
        }
        void myGoToStopEngineer_handle() {
            goal.header.stamp = this->now();
            goal.header.frame_id = "map";
            goal.pose.position.x = S1_Stop_Engineer_point[0].pose.position.x;
            goal.pose.position.y = S1_Stop_Engineer_point[0].pose.position.y;
            goal.pose.position.z = S1_Stop_Engineer_point[0].pose.position.z;
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;
            setState(std::make_shared<GoAndStayState>(this));
        }
        void myGoToStopHero_handle() {
            Patro_points = S1_Stop_Hero_points;
            random = S1_Stop_Hero_points.begin();
            setState(std::make_shared<PatrolState>(this));
        }
        void myS2GoToOutpose() {
            goal.header.stamp = this->now();
            goal.header.frame_id = "map";
            goal.pose.position.x = S2_Outpose_point[0].pose.position.x;
            goal.pose.position.y = S2_Outpose_point[0].pose.position.y;
            goal.pose.position.z = S2_Outpose_point[0].pose.position.z;
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;
            setState(std::make_shared<GoAndStayState>(this));
        }
        void myS3Patro(){
            Patro_points = S3_Patro_points;
            random = S3_Patro_points.begin();
            setState(std::make_shared<PatrolState>(this));
            }

        void myaddhp_handle() {
            std::cout << "addhp_handle is called" << std::endl;
            goal.header.stamp = this->now();
            goal.header.frame_id = "map";
            goal.pose.position.x = self_addhp_point[0].pose.position.x;
            goal.pose.position.y = self_addhp_point[0].pose.position.y;
            goal.pose.position.z = self_addhp_point[0].pose.position.z;
            goal.pose.orientation.x = 0.0;
            goal.pose.orientation.y = 0.0;
            goal.pose.orientation.z = 0.0;
            goal.pose.orientation.w = 1.0;
            setState(std::make_shared<GoAndStayState>(this));
            
            if (!addhptimer) {
                    start_time2 = std::chrono::steady_clock::now();
                    addhptimer = true;
                }

            auto now = std::chrono::steady_clock::now();
                if (std::chrono::duration_cast<std::chrono::seconds>(now - start_time2).count() >= 20) {
                    addhpfail = true;
                    addhptimer = false;
                }
        }

        void mydefend_handle() {
            if (!defend_order_goal_reached) {
                std::cout << "defend_handle is called" << std::endl;
                goal.header.stamp = this->now();
                goal.header.frame_id = "map";
                goal.pose.position.x = self_base_point[0].pose.position.x;
                goal.pose.position.y = self_base_point[0].pose.position.y;
                goal.pose.position.z = self_base_point[0].pose.position.z;
                goal.pose.orientation.x = 0.0;
                goal.pose.orientation.y = 0.0;
                goal.pose.orientation.z = 0.0;
                goal.pose.orientation.w = 1.0;
                setState(std::make_shared<GoAndStayState>(this));
            } else
                setState(std::make_shared<MoveState>(this));
        }

        void myattack_handle() {
            std::cout << "attack_handle is called" << std::endl;
            setState(std::make_shared<AttackState>(this));
        }

        void myguard_handle() {
            if(!shangpofail){
                Patro_points = Guard_points;
                random = Guard_points.begin();
            }
            else if(shangpofail){
                Patro_points = Guard_points2;
                random = Guard_points2.begin();
            }

            setState(std::make_shared<PatrolState>(this));
        }

        void mybuytorelive_handle() {
            rm_decision_interfaces::msg::ToSerial msg;
            if(not_buy_but_relive==1){
                msg.sentry_cmd |= (1 << 31);
            }

            if(not_buy_but_relive== 1 && goldcoin >= buy_to_relive_goldcoin_threshold){
                msg.sentry_cmd |= (1 << 30);
            }

            sentry_cmd_pub_->publish(msg);
            // std::bitset<32> binary(msg.sentry_cmd);
        }

        void myBuyAmmo_handle(){
            buy_ammo_ordered = true;
            if(!ammoBought){
                buy_ammo_num = buy_ammo_num + buy_ammo_num_at_a_time;
                ammoBought = true;
            }
            rm_decision_interfaces::msg::ToSerial msg;
            buy_ammo_num = buy_ammo_num << (31 - 12); 
            msg.sentry_cmd = msg.sentry_cmd | buy_ammo_num;

            sentry_cmd_pub_->publish(msg);
        }

        void mybuyammoremotely_handle() {
            if(!ammoBoughtremotely){
                remote_buy_ammo_times = remote_buy_ammo_times + 1;
                ammoBoughtremotely = true;
            }

            rm_decision_interfaces::msg::ToSerial msg;
            remote_buy_ammo_times = remote_buy_ammo_times << (31 - 16);
            msg.sentry_cmd = msg.sentry_cmd | remote_buy_ammo_times;
            sentry_cmd_pub_->publish(msg);
        }

        void mybuyhp_handle() {
            if(!hpBought){
                buy_hp_times = buy_hp_times + 1;
                hpBought = true;
            }

            rm_decision_interfaces::msg::ToSerial msg;
            buy_hp_times = buy_hp_times << (31 - 20);
            msg.sentry_cmd = msg.sentry_cmd | buy_hp_times;
            sentry_cmd_pub_->publish(msg);
        }

        BT::NodeStatus wait_for_start() {
            if (gamestart) {
                return BT::NodeStatus::FAILURE;
            } else {
                return BT::NodeStatus::SUCCESS;
            }
        }

        BT::NodeStatus IfAsked() {
            return BT::NodeStatus::FAILURE;
        }




        BT::NodeStatus IfAddHp() {
            if (self_hp <= addhp_threshold) {
                addhp_ordered = true;
            }
            if (self_hp == addhp_full_threshold) {
                addhp_ordered = false;
            }
            if(!addhpfail){
                if (addhp_ordered || buy_ammo_ordered) {
                    return BT::NodeStatus::SUCCESS;
                }
                else {
                    return BT::NodeStatus::FAILURE;
                }
            }
            else{
                return BT::NodeStatus::FAILURE;
            }
        }

        BT::NodeStatus IfDefend() {
             if (self_base <= defend_threshold) {
                 return BT::NodeStatus::SUCCESS;
             } else {
                 return BT::NodeStatus::FAILURE;
             }
        }

        BT::NodeStatus IfAttack() {
            if (self_hp >= 1000 && distence(enemypose) <= 3.0) {
                return BT::NodeStatus::SUCCESS;
            } else {
                return BT::NodeStatus::FAILURE;
            }
        }

        BT::NodeStatus IfGuard() {
            if(nav_state == 1){
                addhpfail = false;
            }
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus IfBuyToRelive() {
            if (self_hp == 0 ) {
                return BT::NodeStatus::SUCCESS;
            } else return BT::NodeStatus::FAILURE;
        };

        BT::NodeStatus IfBuyAmmoRemotely() {
            if (self_hp >= buy_ammo_remotely_hp_threshold_when_youdi && goldcoin > buy_ammo_remotely_goldcoin_threshold_when_youdi && self_ammo < buy_ammo_remotely_ammo_threshold_when_youdi) {
                return BT::NodeStatus::SUCCESS;
            }
            else if (self_ammo < buy_ammo_remotely_ammo_threshold_when_wudi && goldcoin > buy_ammo_remotely_goldcoin_threshold_when_wudi && self_outpost >0) {
                return BT::NodeStatus::SUCCESS;

            }
            else{
                ammoBoughtremotely = false;
                return BT::NodeStatus::FAILURE;
            }
        }

        BT::NodeStatus IfBuyHp() {
            if(self_hp < buy_hp_remotely_hp_threshold && goldcoin > buy_hp_remotely_goldcoin_threshold){
                return BT::NodeStatus::SUCCESS;
            }
            else {
                hpBought = false;
                return BT::NodeStatus::FAILURE;
            }

        }

        BT::NodeStatus IfBuyAmmo() {
            if(self_ammo < buy_ammo_local_threshold && goldcoin > buy_ammo_local_goldcoin_threshold){
                return BT::NodeStatus::SUCCESS;
            }
            else {
                ammoBought = false;
                buy_ammo_ordered = false;
                return BT::NodeStatus::FAILURE;
            }
        }


        BT::NodeStatus IfGoToEnemyOutpose() {
            if(enemy_outpost_hp > 0){
                return BT::NodeStatus::SUCCESS;
            }
            else return BT::NodeStatus::FAILURE;
        }

        BT::NodeStatus IfGoToStopEngineer() {
            if(stop_engineer_finished){
                return BT::NodeStatus::FAILURE;
            }
            else return BT::NodeStatus::SUCCESS;
        }
                    
        BT::NodeStatus IfGoToStopHero() {
            return BT::NodeStatus::SUCCESS;
        }
        BT::NodeStatus IfOutposeAlive() {
            if(self_outpost >= self_outpose_threshold){
                return BT::NodeStatus::SUCCESS;
            }
            else {
                sentry_status = 0;
                return BT::NodeStatus::FAILURE;
            }
        }
        BT::NodeStatus S1() {
            if(strategy==1){
                return BT::NodeStatus::SUCCESS;
            }
            else return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus S2() {
            if(strategy==2) {
                return BT::NodeStatus::SUCCESS;
            }
            else return BT::NodeStatus::FAILURE;
        }
        BT::NodeStatus S3() {
            if(strategy==3) {
                return BT::NodeStatus::SUCCESS;
            }
            else return BT::NodeStatus::FAILURE;
        }



        BT::NodeStatus Gimbal_handle() {
            mygimbal_handle();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus addhp_handle() {
            myaddhp_handle();
            return BT::NodeStatus::SUCCESS;
        }
            


        BT::NodeStatus defend_handle() {
            mydefend_handle();
            if (nav_state == 1) {
                defend_order_goal_reached = false;
            }                 
                return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus attack_handle() {
            myattack_handle();

            return BT::NodeStatus::SUCCESS;
 
        }

        BT::NodeStatus Guard() {
            myguard_handle();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus BuyToRelive_handle() {
            mybuytorelive_handle();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus BuyAmmoRemotely_handle() {
            mybuyammoremotely_handle();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus BuyHp_handle() {
            mybuyhp_handle();
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus GoToEnemyOutpose_handle() {
            myGoToEnemyOutpose_handle();
            if(nav_state ==1)
                sentry_status = 2;
            return BT::NodeStatus::SUCCESS;
        }
        BT::NodeStatus BuyAmmo_handle() {
            myBuyAmmo_handle();
            return BT::NodeStatus::SUCCESS;
        }
        BT::NodeStatus GoToStopEngineer_handle() {
            myGoToStopEngineer_handle();
            sentry_status = 1;
            if (nav_state == 1) {
                    stop_engineer_goal_reached = true;
                    std::this_thread::sleep_for(std::chrono::seconds(time_to_stop_enginner));
                    stop_engineer_finished = true;
                }
                    return BT::NodeStatus::SUCCESS;
        }
        
        BT::NodeStatus GoToStopHero_handle() {
            myGoToStopHero_handle();
            sentry_status = 1;
            return BT::NodeStatus::SUCCESS;
        }

        BT::NodeStatus S2GoToOutpose() {
            myS2GoToOutpose();
            if(nav_state == 1){
                sentry_status = 2;
            }
            else{
                sentry_status = 1 ;
            }
            return BT::NodeStatus::SUCCESS;
        }
        BT::NodeStatus S3Patro() {
            myS3Patro();
            sentry_status = 1;
            return BT::NodeStatus::SUCCESS;
        }

        //above is used for bt
    };
}

#endif // RM_DECISION__COMMANDER_HPP_