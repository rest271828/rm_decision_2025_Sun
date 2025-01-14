#include "decision_base/decision_base.hpp"
using namespace RMDecision;

class FSMSolution : public DecisionBase {
public:
    explicit FSMSolution(const rclcpp::NodeOptions& options);

    ~FSMSolution() override;

private:
    void to_target_point(const PlaneCoordinate& point);
    void to_target_point(double x, double y);
};