#pragma once
#include "pff/pff.h"
#include <optional>

namespace flask {
template<int N,int R> class BVP {
public:
    using State=rrtpff::FlatState<N,R>;
    using Curve=rrtpff::FlatTrajectory<N,R>;
    using Weight=Eigen::Matrix<double,N,N>;
    using Boundary=rrtpff::TerminalConstraint<N,R>;
    explicit BVP(Weight weight=Weight::Identity(),double rho=1)
        : rho_(rho),solver_(scaledWeight(weight,rho)) {}
    std::optional<Curve> fixed(const State &from,const State &to,double T) const {
        if constexpr(R==2) {
            if(!from.finite() || !to.finite() || !std::isfinite(T) || T<=0) return {};
            Curve curve; curve.T=T; curve.start=from; curve.end=to;
            const auto delta=(to.derivative(0)-from.derivative(0)).eval();
            const auto v0=from.derivative(1),v1=to.derivative(1);
            curve.coefficients.col(0)=from.derivative(0);
            curve.coefficients.col(1)=v0;
            curve.coefficients.col(2)=(3*delta-(2*v0+v1)*T)/(T*T);
            curve.coefficients.col(3)=(-2*delta+(v0+v1)*T)/(T*T*T);
            const auto w=weight();
            const auto slope=(6*curve.coefficients.col(3)).eval();
            const auto mid=(2*curve.coefficients.col(2)+.5*T*slope).eval();
            // Equivalent integral, expressed as nonnegative quadratic forms
            // to avoid cancellation for almost constant-velocity motions.
            const double energy=T*(mid.dot(w*mid)+(T*T/12)*slope.dot(w*slope));
            curve.cost=std::max(0.,energy)+rho_*T;
            if(!curve.coefficients.allFinite() || !std::isfinite(curve.cost) || energy < -1e-7) return {};
            return curve;
        }
        auto result=solver_.fixedTime(from,Boundary::full(to),T);
        if(!result.success) return {};
        result.trajectory.cost*=rho_;
        return result.trajectory;
    }
    // Energy is a Laurent polynomial. Isolate ALL positive stationary
    // times below its Cauchy root bound, then choose the lowest cost.
    // No sampled candidate-time list or arbitrary maximum time is used.
    std::optional<Curve> optimal(const State &from,const State &to) const {
        if(!from.finite() || !to.finite()) return {};
        const auto boundary=Boundary::full(to);
        typename rrtpff::PFFSolver<N,R>::Energy energy{};
        if constexpr(R==2) {
            const auto delta=(to.derivative(0)-from.derivative(0)).eval();
            const auto v0=from.derivative(1),v1=to.derivative(1);
            const auto w=solver_.weight();
            energy[0]=4*(v0.dot(w*v0)+v0.dot(w*v1)+v1.dot(w*v1));
            energy[1]=-12*(v0+v1).dot(w*delta);
            energy[2]=12*delta.dot(w*delta);
        } else energy=solver_.energyCoefficients(from,boundary);
        auto poly=rrtpff::PFFSolver<N,R>::timePolynomial(energy);
        long double bound=1;
        for(std::size_t k=0;k+1<poly.size();++k) {
            if(!std::isfinite(poly[k])) return {};
            bound=std::max(bound,1+std::abs(poly[k]/poly.back()));
        }
        if(!std::isfinite(bound) || bound>1e100L) return {};
        const auto times=rrtpff::realRootsInInterval(poly,0,double(bound));
        std::optional<Curve> best;
        for(double T:times) {
            if(T<=1e-9 || !std::isfinite(T)) continue;
            // Filter approximate repeated roots by the original dC/dT,
            // avoiding false zeros introduced by multiplication by T^(2r).
            long double derivative=1,scale=1,power=1/T;
            for(std::size_t k=0;k<energy.size();++k) {
                power/=T;
                const long double term=(k+1)*energy[k]*power;
                derivative-=term; scale+=std::abs(term);
            }
            if(std::abs(derivative)>1e-7L*scale) continue;
            auto curve=fixed(from,to,T);
            if(curve && (!best || curve->cost<best->cost)) best=std::move(curve);
        }
        return best;
    }
    Weight weight() const { return solver_.weight()*rho_; }
    double rho() const { return rho_; }
private:
    static Weight scaledWeight(const Weight &w,double rho) {
        if(!std::isfinite(rho)||rho<=0) throw std::invalid_argument("rho must be positive");
        return w/rho;
    }
    double rho_;
    rrtpff::PFFSolver<N,R> solver_;
};
} // namespace flask
