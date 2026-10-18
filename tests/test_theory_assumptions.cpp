#include "theory_test_support.h"
#include "df/unicycle_df.h"
#include "df/paper_r2_df.h"
using namespace theory;
using namespace rrtpff;

void unicycle() {
    UnicycleDFConfig c;
    c.allow_reverse=true;
    UnicycleDF::FlatState z;
    UnicycleDF::FlatControl w1,w2;
    z.values << 0,0,0.2,0;
    w1.values << 0,0; w2.values << 0,0.02;
    UnicycleTrajectoryMetadata metadata;
    require(UnicycleDF::makeMetadata(0.2,metadata,c),"forward branch metadata");
    UnicycleDF::PhysicalState x1,x2;
    UnicycleDF::PhysicalControl u1,u2;
    require(UnicycleDF::flatToPhysicalState(z,w1,metadata,x1,c) &&
            UnicycleDF::flatToPhysicalState(z,w2,metadata,x2,c),"nonzero inverse mapping");
    require(x1.isApprox(x2,0),"Assumption 1: x independent of r=2 pseudo-control");
    require(UnicycleDF::flatToPhysicalControl(z,w1,metadata,u1,c) &&
            UnicycleDF::flatToPhysicalControl(z,w2,metadata,u2,c),"control inverse mapping");
    relative(u2(1)-u1(1),0.1,1e-12,"omega depends on acceleration outside r=2 z");
    // State inversion alone does not imply Definition 4 for bounded omega.
    w2.values(1)=0.2;
    require(!UnicycleDF::sampleFeasible(z,w2,metadata,x2,u2,c),
            "same valid z can violate omega limit with another w");
    UnicycleLift lift; lift.signed_velocity=0.2;
    UnicycleDF::FlatState roundtrip;
    require(UnicycleDF::stateToFlat(x1,lift,roundtrip,c) && roundtrip.values.isApprox(z.values,1e-12),
            "nonzero forward roundtrip");
    metadata.branch=VelocityBranch::Reverse; metadata.signed_start_velocity=-0.2;
    require(UnicycleDF::flatToPhysicalState(z,w1,metadata,x2,c),"nonzero reverse inverse");
    lift.signed_velocity=-0.2;
    require(UnicycleDF::stateToFlat(x2,lift,roundtrip,c) && roundtrip.values.isApprox(z.values,1e-12),
            "nonzero reverse roundtrip with fixed branch");
    metadata.branch=VelocityBranch::Forward; metadata.signed_start_velocity=0.2;
    const double vmin=0.1;
    // A compact angle chart away from the atan2 seam; a deterministic local
    // Lipschitz check is evidence on this set, not a global proof.
    for(double speed:{0.1,0.2,0.4}) for(double angle:{-1.0,-0.3,0.3,1.0}) {
        z.values << 0,0,speed*std::cos(angle),speed*std::sin(angle);
        auto nearby=z; nearby.values(2)+=1e-7;
        require(UnicycleDF::flatToPhysicalState(z,w1,metadata,x1,c) &&
                UnicycleDF::flatToPhysicalState(nearby,w1,metadata,x2,c),"local inverse");
        require((x1-x2).norm() <= 2/vmin*(z.values-nearby.values).norm(),
                "Lipschitz sample on a chart with speed bounded below");
    }
    double previousRatio=0;
    for(double epsilon:{1e-3,1e-4,1e-5}) {
        z.values << 0,0,epsilon,0;
        auto other=z; other.values << 0,0,0,epsilon;
        require(UnicycleDF::flatToPhysicalState(z,w1,metadata,x1,c) &&
                UnicycleDF::flatToPhysicalState(other,w1,metadata,x2,c),"approach zero speed");
        const double ratio=(x1-x2).norm()/(z.values-other.values).norm();
        require(ratio>1000 && ratio>previousRatio,"inverse derivative grows near zero");
        relative(std::abs(x1(2)-x2(2)),std::acos(-1.0)/2,1e-12,
                 "different velocity directions have different heading limits");
        previousRatio=ratio;
    }
    z.values.setZero();
    require(!UnicycleDF::flatToPhysicalState(z,w1,metadata,x1,c),"zero speed singularity rejected");
    z.values << 0,0,-0.2,1e-8;
    auto other=z; other.values(3)=-1e-8;
    require(UnicycleDF::flatToPhysicalState(z,w1,metadata,x1,c) &&
            UnicycleDF::flatToPhysicalState(other,w1,metadata,x2,c),"atan2 seam samples");
    require(std::abs(x1(2)-x2(2))>6,"wrapped real-valued heading has a chart discontinuity");
    const double angularDifference=std::atan2(std::sin(x1(2)-x2(2)),std::cos(x1(2)-x2(2)));
    require(std::abs(angularDifference)<1e-6,"S1 angular metric removes chart seam");
}

Quad3DDF::FlatState fullJet(const Quad3DR2DF::FlatTrajectory &tr,double t) {
    Quad3DDF::FlatState jet;
    for(int d=0;d<4;++d) jet.setDerivative(d,tr.derivative(t,d));
    return jet;
}
void quad3d() {
    static_assert(Quad3DR2DF::FlatState::IntegratorOrder==2);
    static_assert(Quad3DDF::FlatState::IntegratorOrder==4);
    Quad3DDFConfig c;
    Quad3DDF::FlatControl snap; snap.values.setZero();
    Quad3DR2DF::FlatTrajectory a,b; a.T=b.T=1;
    b.coefficients(0,2)=0.5; // Same y/dy at t=0, different acceleration.
    require(a.evaluate(0).values.isApprox(b.evaluate(0).values,0),"identical reduced nodes");
    Quad3DDF::PhysicalState xa,xb;
    require(Quad3DDF::flatToPhysicalState(fullJet(a,0),snap,{},xa,c) &&
            Quad3DDF::flatToPhysicalState(fullJet(b,0),snap,{},xb,c),"cubic edge reconstruction");
    require((xa.segment<4>(3)-xb.segment<4>(3)).norm()>0.01,
            "same reduced z has different attitudes: no alpha(z) exists");
    b.coefficients.setZero(); b.coefficients(0,3)=0.1;
    require(Quad3DDF::flatToPhysicalState(fullJet(b,0),snap,{},xb,c),"nonzero jerk reconstruction");
    require((xa.tail<3>()-xb.tail<3>()).norm()>0.01,
            "same reduced z and acceleration have different angular velocities");
    auto map=std::make_shared<WrenchControlMap>();
    map->lower.setConstant(-100); map->upper.setConstant(100); c.control_map=map;
    const auto jet=fullJet(a,0);
    Quad3DDF::PhysicalControl ua,ub;
    require(Quad3DDF::flatToPhysicalControl(jet,snap,{},ua,c),"hover control reconstruction");
    snap.values(0)=1;
    require(Quad3DDF::flatToPhysicalState(jet,snap,{},xb,c),"r=4 state reconstruction");
    require(xa.isApprox(xb,1e-12),"r=4 state independent of snap");
    require(Quad3DDF::flatToPhysicalControl(jet,snap,{},ub,c),"snap torque reconstruction");
    require((ua-ub).norm()>1e-8,"motor/torque bounds depend on snap outside r=4 z");
    // r>=5 puts snap into z when complete motor/torque constraints are
    // represented as state constraints under Definition 4.
    auto singular=jet; auto acc=singular.derivative(2);
    acc.head<3>() << 0,0,-c.gravity; singular.setDerivative(2,acc);
    require(!Quad3DDF::flatToPhysicalState(singular,snap,{},xb,c),"zero thrust singularity");
    acc.head<3>() << 1,0,-c.gravity; singular.setDerivative(2,acc);
    require(!Quad3DDF::flatToPhysicalState(singular,snap,{},xb,c),"heading/thrust alignment singularity");
    // C1 cubic junction, with acceleration jump -> physical attitude jump.
    a.coefficients.setZero(); a.coefficients(0,1)=0.2;
    b.coefficients.setZero(); b.coefficients(0,0)=0.2;
    b.coefficients(0,1)=0.2; b.coefficients(0,2)=0.5;
    require(a.evaluate(1).values.isApprox(b.evaluate(0).values,1e-12),"C1 reduced junction");
    snap.values.setZero();
    require(Quad3DDF::flatToPhysicalState(fullJet(a,1),snap,{},xa,c) &&
            Quad3DDF::flatToPhysicalState(fullJet(b,0),snap,{},xb,c),"junction edge states");
    require((xa.segment<4>(3)-xb.segment<4>(3)).norm()>0.01,"C1 is insufficient for full physical continuity");
}
void scalesAndSampling() {
    // Unit constants are a symbolic fixture, never estimated DynoBench constants.
    for(const auto nr:{std::pair<int,int>{2,2},{4,2},{4,4}}) {
        const auto [n,r]=nr;
        const double D=(r*n+r*r*n)/2.0, kappa=1, Cp=2;
        double last=std::numeric_limits<double>::infinity();
        for(double N:{10,100,1000,10000,1000000}) {
            const double zeta=std::pow((1+kappa)*std::log(N)/N,1/D);
            const double eta=Cp*zeta;
            require(zeta>0 && zeta<last && eta>0,"Eq.33 shrinking scales for N>e");
            relative(eta/zeta,Cp,1e-12,"eta is Cp*zeta"); last=zeta;
        }
    }
    // Analytic trajectory x(t)=t and a thin closed obstacle interval. This
    // tests the finite-sampling limitation in Algorithm 3, not a full
    // DynoBench collision backend.
    auto collision=[](double t) { return t>=0.049 && t<=0.051; };
    bool coarseHit=false,fineHit=false;
    for(int i=0;i<=10;++i) coarseHit=coarseHit || collision(i/10.0);
    for(int i=0;i<=1000;++i) fineHit=fineHit || collision(i/1000.0);
    require(!coarseHit && fineHit && collision(0.05),"thin obstacle tunneling counterexample");
    // Eq.31 measures directed distance to the trace, not same-time tracking.
    double synchronous=0, directed=0;
    for(int i=0;i<=100;++i) {
        const double t=i/100.0, sigma=t*t, closestOnPi=sigma;
        synchronous=std::max(synchronous,std::abs(sigma-t));
        directed=std::max(directed,std::abs(sigma-closestOnPi));
    }
    require(directed==0 && synchronous>0.2,"Eq.31 is independent of time parameterization");
}
int main() {
    return run([] { unicycle(); quad3d(); scalesAndSampling();
        std::cout << "Nonzero branches, zero-speed limits, reduced jets, snap constraints, "
                     "singular frames, junctions, symbolic scales, thin obstacles checked\n"; });
}
