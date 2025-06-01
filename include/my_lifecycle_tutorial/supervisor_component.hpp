#pragma once

#include <cmath>

namespace lifecycle_supervisor
{

struct Pose2D {
    double x;
    double y;
    double theta;
};

class SupervisorComponent
{
public:
    // 2点間距離計算
    static double CalcDistance(const Pose2D& a, const Pose2D& b);

    // 閾値判定：十分近ければtrue
    static bool IsWithinThreshold(const Pose2D& a, const Pose2D& b, double threshold);
    
    // 追加でスカラー値比較や他の演算が必要なら随時追加
};

} // namespace lifecycle_supervisor
