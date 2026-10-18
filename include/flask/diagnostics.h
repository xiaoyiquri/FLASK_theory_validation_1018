#pragma once
#include "flask/model.h"
#include "common/boundary_diagnostics.h"
#include <Eigen/Geometry>

namespace flask {
inline double attitudeError(const Eigen::VectorXd &a,const Eigen::VectorXd &b,Robot robot) {
    if(robot!=Robot::Integrator) return rrtpff::physicalBoundaryError(a,b).attitude;
    return 0;
}
inline Eigen::Vector3d angularVelocity(const PhysicalSample &s,Robot robot) {
    if(robot==Robot::Quad3D) return s.state.tail<3>();
    if(robot==Robot::Quad2D) return Eigen::Vector3d(0,0,s.state(5));
    if(robot==Robot::Unicycle) return Eigen::Vector3d(0,0,s.control(1));
    return Eigen::Vector3d::Zero();
}
struct BoundaryDiagnostics {
    bool available=false,motion_state_available=false;
    double position=0,velocity=0,attitude=0,angular_velocity=0,full_state=0;
};
struct PathDiagnostics {
    bool available=false;
    double position=0,velocity=0,acceleration=0,jerk=0,attitude=0,angular_velocity=0,full_state=0;
    BoundaryDiagnostics start,goal;
};
template<int N,int R> PathDiagnostics diagnose(const Model<N,R> &model,
        const std::vector<typename BVP<N,R>::Curve> &curves) {
    PathDiagnostics out;
    if(curves.empty()) return out;
    const auto &cfg=model.config(); const int dim=cfg.spatial_dim;
    auto boundary=[&](const PhysicalSample &p,const Eigen::VectorXd &reference) {
        BoundaryDiagnostics b; if(reference.size()!=p.state.size()) return b;
        b.available=true; b.full_state=model.physicalError(p.state,reference);
        b.position=(p.state.head(dim)-reference.head(dim)).norm();
        b.attitude=attitudeError(p.state,reference,cfg.robot);
        if(cfg.robot==Robot::Quad2D) {
            b.motion_state_available=true;
            b.velocity=(p.state.segment<2>(3)-reference.segment<2>(3)).norm();
            b.angular_velocity=std::abs(p.state(5)-reference(5));
        } else if(cfg.robot==Robot::Quad3D) {
            b.motion_state_available=true;
            b.velocity=(p.state.segment<3>(7)-reference.segment<3>(7)).norm();
            b.angular_velocity=(p.state.tail<3>()-reference.tail<3>()).norm();
        }
        return b;
    };
    PhysicalSample previous,first,last;
    for(std::size_t i=0;i<curves.size();++i) {
        const auto &c=curves[i];
        if(!model.sample(c,0,first) || !model.sample(c,c.T,last))
            throw std::runtime_error("Invalid physical endpoint during diagnostics");
        if(i) {
            const auto &left=curves[i-1];
            double *values[]={&out.position,&out.velocity,&out.acceleration,&out.jerk};
            for(int d=0;d<4;++d) *values[d]=std::max(*values[d],
                (left.derivative(left.T,d).head(dim)-c.derivative(0,d).head(dim)).norm());
            out.attitude=std::max(out.attitude,attitudeError(previous.state,first.state,cfg.robot));
            out.angular_velocity=std::max(out.angular_velocity,
                (angularVelocity(previous,cfg.robot)-angularVelocity(first,cfg.robot)).norm());
            out.full_state=std::max(out.full_state,model.physicalError(previous.state,first.state));
        } else out.start=boundary(first,cfg.physical_start);
        previous=last;
    }
    out.goal=boundary(last,cfg.physical_goal); out.available=true; return out;
}
} // namespace flask
