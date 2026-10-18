#include "fixtures.h"
#include "flask/diagnostics.h"
#include <Eigen/Geometry>
using namespace theory;
template<int N,int R> void dynamics(flask::Robot robot) {
    auto c=fixture<N,R>(robot);
    flask::Model<N,R> model(c); flask::BVP<N,R> bvp;
    typename flask::BVP<N,R>::State a,b;
    b.values(0)=.15;
    if(robot==flask::Robot::Unicycle) { a.values(2)=.2; b.values(2)=.2; }
    const auto curve=bvp.optimal(a,b); require(bool(curve),"robot BVP");
    const double t=curve->T*.4,h=1e-5;
    flask::PhysicalSample point,left,right;
    require(model.sample(*curve,t,point)&&model.sample(*curve,t-h,left)&&model.sample(*curve,t+h,right),"physical reconstruction and actuator bounds");
    if(robot==flask::Robot::Quad3D && left.state.segment<4>(3).dot(right.state.segment<4>(3))<0) right.state.segment<4>(3)*=-1;
    const Eigen::VectorXd derivative=(right.state-left.state)/(2*h);
    Eigen::VectorXd expected=Eigen::VectorXd::Zero(point.state.size());
    if(robot==flask::Robot::Unicycle) {
        expected<<point.control(0)*std::cos(point.state(2)),point.control(0)*std::sin(point.state(2)),point.control(1);
    } else if(robot==flask::Robot::Quad2D) {
        const double f1=point.control(0)*c.quad2.mass*c.quad2.gravity/2;
        const double f2=point.control(1)*c.quad2.mass*c.quad2.gravity/2;
        expected<<point.state(3),point.state(4),point.state(5),
            -(f1+f2)*std::sin(point.state(2))/c.quad2.mass,
            (f1+f2)*std::cos(point.state(2))/c.quad2.mass-c.quad2.gravity,
            c.quad2.arm_length*(f1-f2)/c.quad2.inertia;
    } else {
        const Eigen::Quaterniond q(point.state(6),point.state(3),point.state(4),point.state(5));
        const Eigen::Vector3d omega=point.state.tail<3>();
        const Eigen::Quaterniond spin(0,omega.x(),omega.y(),omega.z());
        const auto qdot=q*spin;
        const double arm=.7071067811865475244*c.quad3.arm_length,k=c.quad3.thrust_to_torque;
        Eigen::Matrix4d allocation;
        allocation<<1,1,1,1,-arm,-arm,arm,arm,-arm,arm,arm,-arm,-k,k,-k,k;
        const Eigen::Vector4d wrench=allocation*point.control*(c.quad3.mass*c.quad3.gravity/4);
        expected.head<3>()=point.state.segment<3>(7);
        expected.segment<4>(3)=qdot.coeffs()/2;
        expected.segment<3>(7)=q.toRotationMatrix().col(2)*wrench(0)/c.quad3.mass-Eigen::Vector3d(0,0,c.quad3.gravity);
        expected.tail<3>()=(wrench.tail<3>()-omega.cross(c.quad3.inertia.cwiseProduct(omega))).cwiseQuotient(c.quad3.inertia);
    }
    require((derivative-expected).norm()<2e-4,"independent original nonlinear dynamics residual");
    flask::Validator<N,R> validator(model);
    require(validator.path({*curve},a,b),"robot edge and complete flat endpoints");
}
int main() { return run([] {
    dynamics<2,2>(flask::Robot::Unicycle); dynamics<2,3>(flask::Robot::Unicycle);
    dynamics<2,4>(flask::Robot::Quad2D); dynamics<2,5>(flask::Robot::Quad2D);
    dynamics<4,4>(flask::Robot::Quad3D); dynamics<4,5>(flask::Robot::Quad3D);
    // Reduced r2 joins must not silently pass a physical attitude jump.
    auto c=fixture<4,2>(flask::Robot::Quad3D); flask::Model<4,2> reduced(c);
    flask::Validator<4,2> validator(reduced);
    flask::BVP<4,2>::Curve a,b; a.T=b.T=1;
    a.coefficients(0,1)=.2; b.coefficients(0,0)=.2; b.coefficients(0,1)=.2; b.coefficients(0,2)=.05;
    a.start=a.evaluate(0); a.end=a.evaluate(1); b.start=b.evaluate(0); b.end=b.evaluate(1);
    require(!validator.path({a,b},a.start,b.end),"reduced physical discontinuity rejected by default");
    flask::ValidationConfig practical;
    practical.enforce_physical_continuity=false;
    practical.enforce_physical_endpoints=false;
    flask::Validator<4,2> reducedValidator(reduced,practical);
    require(reducedValidator.path({a,b},a.start,b.end),"explicit reduced contract accepts feasible cubic interiors");
    const auto jumps=flask::diagnose(reduced,std::vector<flask::BVP<4,2>::Curve>{a,b});
    require(jumps.available && jumps.position<1e-12 && jumps.velocity<1e-12,"reduced join has continuous position and velocity");
    require(jumps.acceleration>.09 && jumps.attitude>1e-3,"reduced higher-order/attitude discontinuity measured without changing acceptance");
    c.physical_start=Eigen::VectorXd::Zero(13); c.physical_start(6)=1;
    c.physical_goal=c.physical_start; c.physical_goal(0)=.45;
    flask::Model<4,2> boundaryModel(c);
    flask::Validator<4,2> strictBoundary(boundaryModel);
    require(!strictBoundary.path({b},b.start,b.end),"full physical boundaries remain enforced by default");
    flask::Validator<4,2> reducedBoundary(boundaryModel,practical);
    require(reducedBoundary.path({b},b.start,b.end),"reduced endpoint relaxation is explicit");
    auto narrow=fixture<2,2>(flask::Robot::Unicycle);
    narrow.lower << -2,-2,-.01,-.01; narrow.upper=-narrow.lower;
    flask::Model<2,2> samplingOnly(narrow);
    flask::BVP<2,2>::State moving; moving.values<<0,0,.2,0;
    require(samplingOnly.nodeValid(moving),"sampling velocity support must not become an extra physical limit");
    narrow.constrain_sampling_bounds=true;
    flask::Model<2,2> explicitLimits(narrow);
    require(!explicitLimits.nodeValid(moving),"explicit flat constraint option remains enforceable");
    auto stopConfig=fixture<2,2>(flask::Robot::Unicycle);
    flask::Model<2,2> stopModel(stopConfig);
    flask::BVP<2,2>::Curve hiddenStop; hiddenStop.T=1;
    // dy=(t-.37)^2: positive at every .1 grid point, zero at .37.
    hiddenStop.coefficients(0,1)=.37*.37;
    hiddenStop.coefficients(0,2)=-.37;
    hiddenStop.coefficients(0,3)=1./3;
    for(int i=0;i<=10;++i) {
        flask::PhysicalSample point;
        require(stopModel.sample(hiddenStop,i*.1,point),"coarse grid misses exact zero velocity");
    }
    require(!stopModel.intervalValid(hiddenStop),"analytic speed extrema reject unsampled stop");
    hiddenStop.normalized=true; hiddenStop.T=2;
    require(!stopModel.intervalValid(hiddenStop),"speed extrema also handle normalized coefficients");
}); }
