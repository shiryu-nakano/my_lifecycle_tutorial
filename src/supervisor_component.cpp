#include "my_lifecycle_tutorial/supervisor_component.hpp"
#include <cmath>


/*
ロジックの実装
*/
namespace lifecycle_supervisor
{

double SupervisorComponent::CalcDistance(const Pose2D& a, const Pose2D& b)
{
    double dx = a.x - b.x;
    double dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

bool SupervisorComponent::IsWithinThreshold(const Pose2D& a, const Pose2D& b, double threshold)
{
    return CalcDistance(a, b) <= threshold;
}

} // namespace lifecycle_supervisor
