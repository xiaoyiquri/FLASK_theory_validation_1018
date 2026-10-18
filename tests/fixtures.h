#pragma once
#include "theory_test_support.h"
#include "flask/planner.h"
template<int N,int R> flask::ModelConfig fixture(flask::Robot robot=flask::Robot::Integrator) {
    flask::ModelConfig c; c.robot=robot; c.spatial_dim=std::min(N,2); c.radius=0;
    c.lower=Eigen::VectorXd::Constant(N*R,-10); c.upper=-c.lower;
    c.unicycle.max_velocity=2; c.unicycle.max_omega=3;
    if(robot==flask::Robot::Quad3D) c.spatial_dim=3;
    return c;
}
