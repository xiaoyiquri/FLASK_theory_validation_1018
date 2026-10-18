#include "fixtures.h"
using namespace theory;
template<int N,int R> void verify() {
    flask::BVP<N,R> bvp;
    typename flask::BVP<N,R>::State a,b;
    for(int i=0;i<N;++i) b.values(i)=0.1*(i+1);
    const auto curve=bvp.optimal(a,b); require(bool(curve),"global stationary BVP solution");
    require((curve->evaluate(0).values-a.values).norm()<1e-7,"all initial derivatives constrained");
    require((curve->evaluate(curve->T).values-b.values).norm()<1e-6,"all terminal derivatives constrained");
    const double t=curve->T*0.4,h=curve->T*1e-5;
    const Eigen::Matrix<double,N*R,1> numerical=(curve->evaluate(t+h).values-curve->evaluate(t-h).values)/(2*h);
    const auto chain=rrtpff::IntegratorChain<N,R>::system();
    require((numerical-(chain.A*curve->evaluate(t).values+chain.B*curve->control(t))).norm()<1e-5,"flat ODE residual");
    double energy=0;
    const int intervals=1000;
    for(int i=0;i<=intervals;++i) {
        const auto w=curve->control(curve->T*i/intervals);
        energy+=(i==0 || i==intervals ? 1:(i%2 ? 4:2))*w.dot(bvp.weight()*w);
    }
    energy*=curve->T/(3*intervals);
    relative(curve->cost,energy+curve->T,1e-7,"integrated control cost agrees with Gramian energy");
}
int main() { return run([] {
    {
        Eigen::Matrix2d weight;weight<<2,.3,.3,1;
        flask::BVP<2,2> direct(weight,.7);rrtpff::PFFSolver<2,2> oracle(weight/.7);
        flask::BVP<2,2>::State from,to;from.values<<.2,-.3,.4,-.1;to.values<<1,.4,-.2,.3;
        for(double T:{.01,.2,1.,8.}) {
            const auto curve=direct.fixed(from,to,T);
            const auto expected=oracle.fixedTime(from,rrtpff::TerminalConstraint<2,2>::full(to),T);
            require(curve && expected.success,"r2/SPD direct and Gramian BVP exist");
            relative(curve->cost,expected.cost*.7,1e-8,"Eq20 weighted energy agrees with independent Gramian solver");
            require((curve->coefficients-expected.trajectory.coefficients).norm()<1e-5*(1+curve->coefficients.norm()),"Eq25 coefficients agree");
        }
    }
    verify<1,1>(); verify<2,2>(); verify<2,3>(); verify<4,4>(); verify<4,5>();
    using State=flask::BVP<1,2>::State;
    State a,b; b.values(0)=1;
    flask::BVP<1,2> bvp(Eigen::Matrix<double,1,1>::Identity(),4);
    const auto fixed=bvp.fixed(a,b,2); require(bool(fixed),"fixed-time solver");
    relative(fixed->cost,9.5,1e-12,"energy 12/T^3 plus rho*T");
    const auto optimal=bvp.optimal(a,b); require(bool(optimal),"free-time solver");
    relative(optimal->T,std::sqrt(3.0),1e-9,"quartic optimal time with rho=4");
    a.values << 0,2; b.values << 1,2;
    flask::BVP<1,2> multiple;
    const auto curve=multiple.optimal(a,b); require(bool(curve),"multiple positive root case");
    double sampled=INFINITY;
    for(int i=0;i<3000;++i) {
        const double t=std::exp(std::log(.01)+i*std::log(10000.)/2999);
        const auto c=multiple.fixed(a,b,t); if(c) sampled=std::min(sampled,c->cost);
    }
    require(curve->cost<=sampled+1e-7,"choose global minimum over stationary roots");
    const auto forward=multiple.optimal(a,b),backward=multiple.optimal(b,a);
    require(forward && backward && std::abs(forward->cost-backward->cost)>0.1,"steering cost is directed");
    State rest; require(!multiple.optimal(rest,rest),"zero-motion infimum handled without fake positive duration");
    require(!multiple.fixed(a,b,-1),"negative time rejected");
}); }
