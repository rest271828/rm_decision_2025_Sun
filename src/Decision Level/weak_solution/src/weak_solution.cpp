#include "weak_solution/weak_solution.hpp"
#include <random>
using namespace RMDecision;

WeakSolution::WeakSolution(const rclcpp::NodeOptions& options) : DecisionBase(7, "weak_solution", options) {

}

PlaneCoordinate WeakSolution::generate_random_point(double radius) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-radius, radius);
    auto tmp_point = PlaneCoordinate(
        prism_.self.pose.pose.position.x, prism_.self.pose.pose.position.y);
    tmp_point = tmp_point + PlaneCoordinate(dis(gen), dis(gen));
    return tmp_point;
}

void WeakSolution::to_target_point(const PlaneCoordinate& point) {
    nav_to_pose(point.to_pose_stamped(this->now()), false);
}

void WeakSolution::to_target_point(double x, double y) {
    nav_to_pose(PlaneCoordinate(x, y).to_pose_stamped(this->now()), false);
}
