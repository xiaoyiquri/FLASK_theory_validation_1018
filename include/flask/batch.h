#pragma once
#include "flask/model.h"
#include <chrono>

namespace flask {
// Eight times on ONE curve, SoA throughout. No AVX types cross the ABI.
struct PackedCurve {
    int n=0,r=0,degree=0;
    double T=0;
    bool normalized=false;
    double coefficients[5][4][10]{};
};
template<int N,int R> PackedCurve packCurve(const typename BVP<N,R>::Curve &curve) {
    static_assert(N<=4 && R<=5,"Batch ABI supports the six low-DOF models");
    PackedCurve out; out.n=N; out.r=R; out.degree=curve.Degree;
    out.T=curve.T; out.normalized=curve.normalized;
    for(int d=0;d<5;++d) for(int a=0;a<N;++a) for(int k=d;k<=curve.Degree;++k) {
        double factor=1; for(int j=0;j<d;++j) factor*=k-j;
        out.coefficients[d][a][k-d]=curve.coefficients(a,k)*factor*
            (curve.normalized ? std::pow(curve.T,-d):1.);
    }
    return out;
}
struct BatchContext {
    ModelConfig model;
    Eigen::Matrix4d inverse=Eigen::Matrix4d::Identity();
    Eigen::Vector4d control_lower=Eigen::Vector4d::Zero(),control_upper=Eigen::Vector4d::Ones();
    std::vector<double> obstacle_x,obstacle_y,obstacle_z,radius_squared;
    bool parameters_valid=true;
    explicit BatchContext(const ModelConfig &);
};
struct BatchResult {
    alignas(32) double jet[5][4][8]{};
    double control[4][8]{},omega[3][8]{},orientation[9][8]{};
    unsigned physical_valid=0;
    unsigned flat_bound_rejections=0;
    bool collision=false;
    std::array<unsigned,static_cast<std::size_t>(rrtpff::FeasibilityFailure::Count)> failures{};
    double evaluation_seconds=0,physical_seconds=0,collision_seconds=0;
};
// Throws if no AVX2 backend exists; Validator explicitly selects its scalar
// reference on unsupported CPUs. All three robot mappings use double precision.
BatchResult evaluateBatch(const BatchContext &,const PackedCurve &,const double times[8],
                          unsigned active,bool profile=false);
} // namespace flask
