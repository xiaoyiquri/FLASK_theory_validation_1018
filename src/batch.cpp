#include "flask/batch.h"
namespace flask {
BatchContext::BatchContext(const ModelConfig &c):model(c) {
    const auto positive=[](double x){return std::isfinite(x) && x>0;};
    const auto nonnegative=[](double x){return !std::isnan(x) && x>=0;};
    if(c.robot==Robot::Quad2D) {
        const auto &q=c.quad2;
        parameters_valid=positive(q.mass)&&positive(q.gravity)&&positive(q.inertia)&&positive(q.arm_length)&&
            positive(q.max_motor)&&std::isfinite(q.max_velocity)&&nonnegative(q.max_velocity)&&
            nonnegative(q.max_acceleration)&&nonnegative(q.max_jerk)&&q.max_thrust>0&&!std::isnan(q.max_thrust)&&
            std::isfinite(q.max_omega)&&nonnegative(q.max_omega)&&nonnegative(q.max_angular_acceleration)&&
            positive(q.singular_epsilon)&&std::isfinite(q.lift_tolerance)&&nonnegative(q.lift_tolerance);
    }
    if(c.robot==Robot::Quad3D) {
        const auto &q=c.quad3;
        parameters_valid=positive(q.mass)&&positive(q.gravity)&&positive(q.arm_length)&&positive(q.thrust_to_torque)&&
            q.inertia.allFinite()&&(q.inertia.array()>0).all()&&positive(q.max_motor)&&
            std::isfinite(q.max_velocity)&&nonnegative(q.max_velocity)&&nonnegative(q.max_acceleration)&&
            nonnegative(q.max_jerk)&&std::isfinite(q.max_omega)&&nonnegative(q.max_omega)&&
            nonnegative(q.max_angular_acceleration)&&positive(q.singular_epsilon)&&
            std::isfinite(q.lift_tolerance)&&nonnegative(q.lift_tolerance);
    }
    collisionMask({},c.obstacles,c.radius,false);
    for(const auto &s:c.obstacles) {
        obstacle_x.push_back(s.center.x()); obstacle_y.push_back(s.center.y());
        obstacle_z.push_back(s.center.z()); radius_squared.push_back(std::pow(s.radius+c.radius,2));
    }
    if(c.robot==Robot::Quad3D) {
        const auto &q=c.quad3;
        if(q.control_map) { inverse=q.control_map->inverse; control_lower=q.control_map->lower; control_upper=q.control_map->upper; }
        else {
            const double a=.7071067811865475244*q.arm_length,k=q.thrust_to_torque;
            Eigen::Matrix4d allocation;
            allocation<<1,1,1,1,-a,-a,a,a,-a,a,a,-a,-k,k,-k,k;
            allocation*=q.mass*q.gravity/4;
            if(std::abs(allocation.determinant())>1e-30) inverse=allocation.fullPivLu().inverse();
            else parameters_valid=false;
            control_lower.setConstant(-1e-8); control_upper.setConstant(q.max_motor+1e-8);
        }
        parameters_valid=parameters_valid && inverse.allFinite() && control_lower.allFinite() &&
            control_upper.allFinite() && (control_lower.array()<=control_upper.array()).all();
    }
}
#ifdef FLASK_HAS_AVX2
BatchResult evaluateBatchAVX2(const BatchContext &,const PackedCurve &,const double *,unsigned,bool);
#endif
BatchResult evaluateBatch(const BatchContext &context,const PackedCurve &curve,const double times[8],
                          unsigned active,bool profile) {
    if(curve.n<1 || curve.n>4 || curve.r<1 || curve.r>5 || curve.degree!=2*curve.r-1 ||
       !std::isfinite(curve.T) || curve.T<=0 || active>255) throw std::invalid_argument("Invalid packed curve");
    for(int i=0;i<8;++i) if((active&(1u<<i)) && (!std::isfinite(times[i]) || times[i]<0 || times[i]>curve.T))
        throw std::invalid_argument("Invalid batch time");
#ifdef FLASK_HAS_AVX2
    if(avx2Available()) return evaluateBatchAVX2(context,curve,times,active,profile);
#endif
    throw std::runtime_error("AVX2 full trajectory validation is unavailable");
}
} // namespace flask
