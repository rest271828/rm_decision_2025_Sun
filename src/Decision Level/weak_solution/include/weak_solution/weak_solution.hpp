#include "decision_base/decision_base.hpp"
#include <random>
using namespace RMDecision;

class WeakSolution : public DecisionBase {
public:
    explicit WeakSolution(const rclcpp::NodeOptions& options);

    ~WeakSolution() override;

private:
    PlaneCoordinate generate_random_point(double radius);

    void to_target_point(const PlaneCoordinate& point);

    void to_target_point(double x, double y);
};

