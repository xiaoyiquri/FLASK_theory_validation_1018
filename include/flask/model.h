#pragma once
#include "flask/bvp.h"
#include "flask/geometry.h"
#include "df/unicycle_df.h"
#include "df/quad2d_df.h"
#include "df/quad3d_df.h"
#include <string>

namespace flask {
enum class Robot { Integrator,Unicycle,Quad2D,Quad3D };
struct PhysicalSample {
    Eigen::VectorXd state,control;
    Eigen::Vector3d position=Eigen::Vector3d::Zero();
};
struct ModelConfig {
    Robot robot=Robot::Integrator;
    int spatial_dim=2;
    Eigen::VectorXd lower,upper;
    std::vector<Sphere> obstacles;
    double radius=0.1;
    int unicycle_branch=1;
    bool constrain_sampling_bounds=false;
    rrtpff::UnicycleDFConfig unicycle;
    rrtpff::Quad2DDFConfig quad2;
    rrtpff::Quad3DDFConfig quad3;
    Eigen::VectorXd physical_start,physical_goal;
};
template<int N,int R> class Model {
public:
    using State=typename BVP<N,R>::State;
    using Curve=typename BVP<N,R>::Curve;
    explicit Model(ModelConfig config):config_(std::move(config)) {
        if(config_.spatial_dim<1 || config_.spatial_dim>3 || config_.spatial_dim>N ||
           config_.lower.size()!=N*R || config_.upper.size()!=N*R ||
           !config_.lower.allFinite() || !config_.upper.allFinite() ||
           (config_.lower.array()>config_.upper.array()).any() ||
           !std::isfinite(config_.radius) || config_.radius<0)
            throw std::invalid_argument("Invalid model/flat sampling bounds");
        if((config_.robot==Robot::Unicycle && (N!=2 || R<2)) ||
           (config_.robot==Robot::Quad2D && (N!=2 || (R!=2 && R<4))) ||
           (config_.robot==Robot::Quad3D && (N!=4 || (R!=2 && R<4))))
            throw std::invalid_argument("Flat dimensions do not match robot");
        if(config_.unicycle_branch!=1 && config_.unicycle_branch!=-1)
            throw std::invalid_argument("Unicycle branch must be +1 or -1");
        // Validate the entire obstacle set, including empty-active batches.
        collisionMask({},config_.obstacles,config_.radius,false);
    }
    const ModelConfig &config() const { return config_; }
    bool nodeValid(const State &z) const {
        if(!flatValid(z)) return false;
        PositionBatch b; b.active=1;
        b.position[0].head(config_.spatial_dim)=z.values.head(config_.spatial_dim);
        if(collisionMask(b,config_.obstacles,config_.radius,false)) return false;
        if(config_.robot==Robot::Unicycle) {
            if constexpr(N==2 && R>=2) {
                const double v=z.derivative(1).norm();
                return v>config_.unicycle.velocity_epsilon && v<=config_.unicycle.max_velocity;
            }
        }
        if constexpr(N==2 && R>=4) {
            if(config_.robot==Robot::Quad2D) {
                rrtpff::Quad2DDF::FlatState f;
                for(int d=0;d<4;++d) f.setDerivative(d,z.derivative(d));
                rrtpff::Quad2DDF::PhysicalState x; rrtpff::Quad2DDF::FlatControl w;
                return rrtpff::Quad2DDF::flatToPhysicalState(f,w,{},x,config_.quad2) &&
                    z.derivative(1).norm()<=config_.quad2.max_velocity && std::abs(x(5))<=config_.quad2.max_omega;
            }
        }
        if constexpr(N==4 && R>=4) {
            if(config_.robot==Robot::Quad3D) {
                rrtpff::Quad3DDF::FlatState f;
                for(int d=0;d<4;++d) f.setDerivative(d,z.derivative(d));
                rrtpff::Quad3DDF::PhysicalState x; rrtpff::Quad3DDF::FlatControl w;
                return rrtpff::Quad3DDF::flatToPhysicalState(f,w,{},x,config_.quad3) &&
                    z.derivative(1).template head<3>().norm()<=config_.quad3.max_velocity &&
                    x.template tail<3>().norm()<=config_.quad3.max_omega;
            }
        }
        return true;
    }
    bool flatValid(const State &z) const {
        if(!z.finite()) return false;
        // Flat sampling support is not an actuator/state constraint. The
        // physical bounds are enforced by the nonlinear model. Workspace
        // position bounds apply throughout the edge in either mode.
        const int count=config_.constrain_sampling_bounds || config_.robot==Robot::Integrator
            ? N*R:config_.spatial_dim;
        return (z.values.head(count).array()>=config_.lower.head(count).array()-1e-9).all() &&
               (z.values.head(count).array()<=config_.upper.head(count).array()+1e-9).all();
    }
    bool sample(const Curve &curve,double t,PhysicalSample &out,rrtpff::FeasibilityFailure *reason=nullptr,bool *flat_bound=nullptr) const {
        if(reason) *reason=rrtpff::FeasibilityFailure::Numeric;
        if(flat_bound) *flat_bound=false;
        const auto z=curve.evaluateState(t);
        if(!flatValid(z)) { if(flat_bound) *flat_bound=z.finite(); return false; }
        out.position.setZero(); out.position.head(config_.spatial_dim)=z.values.head(config_.spatial_dim);
        if(config_.robot==Robot::Integrator) {
            out.state=z.values; out.control=curve.control(t); return out.control.allFinite();
        }
        if constexpr(N==2 && R>=2) {
            if(config_.robot==Robot::Unicycle) {
                rrtpff::UnicycleDF::FlatState flat;
                flat.setDerivative(0,curve.derivative(t,0)); flat.setDerivative(1,curve.derivative(t,1));
                rrtpff::UnicycleDF::FlatControl acceleration; acceleration.values=curve.derivative(t,2);
                rrtpff::UnicycleTrajectoryMetadata meta;
                if(!rrtpff::UnicycleDF::makeMetadata(config_.unicycle_branch*curve.derivative(0,1).norm(),
                                                    meta,config_.unicycle)) {
                    if(reason) *reason=rrtpff::FeasibilityFailure::Branch;
                    return false;
                }
                rrtpff::UnicycleDF::PhysicalState x; rrtpff::UnicycleDF::PhysicalControl u;
                if(!rrtpff::UnicycleDF::sampleFeasible(flat,acceleration,meta,x,u,config_.unicycle,reason)) return false;
                out.state=x; out.control=u; return true;
            }
            if(config_.robot==Robot::Quad2D) {
                rrtpff::Quad2DDF::FlatState flat;
                for(int d=0;d<4;++d) flat.setDerivative(d,curve.derivative(t,d));
                rrtpff::Quad2DDF::FlatControl snap; snap.values=curve.derivative(t,4);
                rrtpff::Quad2DDF::PhysicalState x; rrtpff::Quad2DDF::PhysicalControl u;
                if(!rrtpff::Quad2DDF::sampleFeasible(flat,snap,{},x,u,config_.quad2,reason)) return false;
                out.state=x; out.control=u; return true;
            }
        }
        if constexpr(N==4 && R>=2) {
            if(config_.robot==Robot::Quad3D) {
                rrtpff::Quad3DDF::FlatState flat;
                for(int d=0;d<4;++d) flat.setDerivative(d,curve.derivative(t,d));
                rrtpff::Quad3DDF::FlatControl snap; snap.values=curve.derivative(t,4);
                rrtpff::Quad3DDF::PhysicalState x; rrtpff::Quad3DDF::PhysicalControl u;
                if(!rrtpff::Quad3DDF::sampleFeasible(flat,snap,{},x,u,config_.quad3,reason)) return false;
                out.state=x; out.control=u; return true;
            }
        }
        return false;
    }
    bool intervalValid(const Curve &curve,rrtpff::FeasibilityFailure *reason=nullptr,bool *flat_bound=nullptr) const {
        if(reason) *reason=rrtpff::FeasibilityFailure::Numeric;
        if(flat_bound) *flat_bound=false;
        if constexpr(N==2 && R==2) {
            if(config_.robot==Robot::Unicycle) {
                // All extrema of ||dy||^2 on normalized time s in [0,1].
                // This checks speed/zero singularities between grid points;
                // angular-rate constraints still use finite sampling.
                std::vector<long double> squared(5,0),slope(4,0);
                for(int axis=0;axis<2;++axis) {
                    long double v[3];
                    for(int k=0;k<3;++k) v[k]=(k+1)*static_cast<long double>(curve.coefficients(axis,k+1))*
                        (curve.normalized ? 1/curve.T:std::pow(static_cast<long double>(curve.T),k));
                    for(int k=0;k<3;++k) for(int j=0;j<3;++j) squared[k+j]+=v[k]*v[j];
                }
                for(int k=1;k<=4;++k) {
                    slope[k-1]=k*squared[k];
                    if(!std::isfinite(slope[k-1])) return false;
                }
                auto critical=rrtpff::realRootsInInterval(slope,0,1);
                critical.push_back(0); critical.push_back(1);
                for(double fraction:critical) {
                    PhysicalSample out;
                    if(!sample(curve,fraction*curve.T,out,reason,flat_bound)) return false;
                }
            }
        }
        return true;
    }
    double physicalError(Eigen::VectorXd a,const Eigen::VectorXd &b) const {
        if(a.size()!=b.size() || !a.allFinite() || !b.allFinite()) return INFINITY;
        if(config_.robot==Robot::Unicycle) a(2)=b(2)+angle(a(2)-b(2));
        if(config_.robot==Robot::Quad2D) a(2)=b(2)+angle(a(2)-b(2));
        if(config_.robot==Robot::Quad3D && a.segment<4>(3).dot(b.segment<4>(3))<0)
            a.segment<4>(3)*=-1;
        return (a-b).norm();
    }
private:
    static double angle(double a) { return std::atan2(std::sin(a),std::cos(a)); }
    ModelConfig config_;
};
} // namespace flask
