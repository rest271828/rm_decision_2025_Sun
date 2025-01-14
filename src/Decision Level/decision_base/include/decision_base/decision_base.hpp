#include "information_widgets/chessboard_def.hpp"
#include "information_widgets/prism_def.hpp"
#include "information_widgets/rm_decision_defs.hpp"
#include "iw_interfaces/msg/chessboard.hpp"
#include "iw_interfaces/msg/prism.hpp"
#include "navigator_interfaces/msg/navigate.hpp"
#include "rclcpp/rclcpp.hpp"

namespace RMDecision{
class DecisionBase : public rclcpp::Node {
public:
    explicit DecisionBase(uint selfId, std::string nodeName, const rclcpp::NodeOptions &options);

private:
    rclcpp::Subscription<iw_interfaces::msg::Chessboard>::SharedPtr chessboard_sub_;
    rclcpp::Subscription<iw_interfaces::msg::Prism>::SharedPtr prism_sub_;
    rclcpp::Publisher<navigator_interfaces::msg::Navigate>::SharedPtr nav_pub_;
    ChessboardHandle chessboard_;
    Prism prism_;

    void chessboard_sub_callback(const iw_interfaces::msg::Chessboard::SharedPtr msg);

    void prism_sub_callback(const iw_interfaces::msg::Prism::SharedPtr msg);

    void nav_to_pose(const Pose& pose, bool instant);
};

}  // namespace RMDecision
