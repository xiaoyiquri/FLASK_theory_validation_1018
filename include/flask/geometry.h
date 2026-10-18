#pragma once
#include <Eigen/Core>
#include <array>
#include <vector>

namespace flask {
struct Sphere { Eigen::Vector3d center=Eigen::Vector3d::Zero(); double radius=0; };
// Eight configurations per call; inactive lanes are never inspected.
struct PositionBatch {
    std::array<Eigen::Vector3d,8> position;
    unsigned active=0;
    PositionBatch() { for(auto &p:position) p.setZero(); }
};
bool avx2Available();
unsigned collisionMask(const PositionBatch &,const std::vector<Sphere> &,double robotRadius,
                       bool useSIMD=true);
} // namespace flask
