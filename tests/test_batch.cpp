#include "fixtures.h"
#include <random>
using namespace theory;
template<int N,int R> void compare(flask::Robot robot,bool wrench=false) {
    auto config=fixture<N,R>(robot);
    if(wrench) {
        auto map=std::make_shared<rrtpff::WrenchControlMap>();
        map->inverse.setZero();map->inverse.diagonal()<<1/(config.quad3.mass*config.quad3.gravity),.05,.05,.05;
        map->lower<<0,-.1,-.1,-.1;map->upper<<1.5,.1,.1,.1;
        config.quad3.control_map=map;
    }
    flask::Model<N,R> model(config); flask::BatchContext context(config);
    std::mt19937 rng(20261017+N+R);std::uniform_real_distribution<double> random(-.05,.05);
    for(int iteration=0;iteration<150;++iteration) {
        typename flask::BVP<N,R>::Curve curve;
        curve.T=.4+.01*iteration;
        for(int a=0;a<N;++a) for(int k=0;k<=curve.Degree;++k) curve.coefficients(a,k)=random(rng);
        if(robot==flask::Robot::Unicycle) curve.coefficients(0,1)+=.3;
        if(iteration%2) {
            for(int k=0;k<=curve.Degree;++k) curve.coefficients.col(k)*=std::pow(curve.T,k);
            curve.normalized=true;
        }
        const auto packed=flask::packCurve<N,R>(curve);
        double times[8];unsigned active=iteration%256;
        for(int i=0;i<8;++i) times[i]=curve.T*(double(i)/7.);
        const auto batch=flask::evaluateBatch(context,packed,times,active,true);
        unsigned expected=0;
        for(int i=0;i<8;++i) if(active&(1u<<i)) {
            for(int d=0;d<5;++d) for(int a=0;a<N;++a)
                require(std::abs(batch.jet[d][a][i]-curve.derivative(times[i],d)(a))<1e-9,"SIMD derivative/normalized-time agreement");
            flask::PhysicalSample scalar;
            const bool valid=model.sample(curve,times[i],scalar);
            if(valid) expected|=1u<<i;
            if(valid && (batch.physical_valid&(1u<<i))) {
                for(int k=0;k<scalar.control.size();++k)
                    require(std::abs(batch.control[k][i]-scalar.control(k))<1e-8,"SIMD physical control agrees with scalar inverse");
                if(robot==flask::Robot::Quad3D) {
                    Eigen::Quaterniond q(scalar.state(6),scalar.state(3),scalar.state(4),scalar.state(5));
                    auto rotation=q.toRotationMatrix();
                    for(int a=0;a<3;++a) for(int b=0;b<3;++b)
                        require(std::abs(rotation(a,b)-batch.orientation[b*3+a][i])<2e-10,"SIMD frame agrees with physical quaternion");
                    for(int k=0;k<3;++k)
                        require(std::abs(scalar.state(10+k)-batch.omega[k][i])<1e-9,"SIMD body rates agree with scalar frame");
                }
            }
        }
        if(expected!=batch.physical_valid) {
            std::cerr<<"mask mismatch n="<<N<<" r="<<R<<" robot="<<int(robot)<<" iteration="<<iteration
                     <<" expected="<<expected<<" actual="<<batch.physical_valid<<'\n';
        }
        require(expected==batch.physical_valid,"scalar/SIMD constraint masks agree");
    }
}
int main(){return run([] {
    if(!flask::avx2Available()) { std::cout<<"SKIP: AVX2 backend unavailable\n"; return; }
    compare<1,1>(flask::Robot::Integrator);compare<2,2>(flask::Robot::Unicycle);
    compare<2,3>(flask::Robot::Unicycle);compare<2,2>(flask::Robot::Quad2D);
    compare<2,4>(flask::Robot::Quad2D);compare<2,5>(flask::Robot::Quad2D);
    compare<4,2>(flask::Robot::Quad3D);compare<4,4>(flask::Robot::Quad3D);
    compare<4,5>(flask::Robot::Quad3D);compare<4,2>(flask::Robot::Quad3D,true);
    auto c=fixture<1,1>(); c.obstacles={{Eigen::Vector3d(.5,0,0),.01}};
    flask::BVP<1,1> bvp;flask::BVP<1,1>::State a,b;b.values(0)=1;
    auto curve=bvp.fixed(a,b,1);double t[8]{.5,0,0,0,0,0,0,0};
    auto hit=flask::evaluateBatch(flask::BatchContext(c),flask::packCurve<1,1>(*curve),t,1);
    require(hit.collision,"SIMD collision checks active configurations");
    t[0]=0;t[7]=NAN;
    require(!flask::evaluateBatch(flask::BatchContext(c),flask::packCurve<1,1>(*curve),t,1).collision,"inactive NaN time ignored");
    for(double invalid:{-.01,1.01,std::numeric_limits<double>::quiet_NaN(),std::numeric_limits<double>::infinity()}) {
        t[0]=invalid; bool rejected=false;
        try { flask::evaluateBatch(flask::BatchContext(c),flask::packCurve<1,1>(*curve),t,1); }
        catch(const std::invalid_argument &) { rejected=true; }
        require(rejected,"active out-of-range/nonfinite time rejected");
    }
    t[0]=0;
    curve->coefficients(0,0)=NAN;
    require(flask::evaluateBatch(flask::BatchContext(c),flask::packCurve<1,1>(*curve),t,1).physical_valid==0,"nonfinite trajectory rejected");
    auto invalid=fixture<2,2>(flask::Robot::Quad2D);invalid.quad2.max_omega=NAN;
    flask::BVP<2,2>::Curve hover;hover.T=1;double ht[8]{};
    require(flask::evaluateBatch(flask::BatchContext(invalid),flask::packCurve<2,2>(hover),ht,255).physical_valid==0,"invalid physical parameters rejected");
    auto uni=fixture<2,2>(flask::Robot::Unicycle);
    uni.unicycle.max_omega=.5;
    flask::Model<2,2> uniModel(uni); flask::BatchContext uniContext(uni);
    for(double delta:{-1e-8,0.,1e-8}) {
        flask::BVP<2,2>::Curve boundary;boundary.T=1;
        boundary.coefficients(0,1)=.3;boundary.coefficients(1,2)=.075+delta;
        double times[8]{};const auto output=flask::evaluateBatch(uniContext,flask::packCurve<2,2>(boundary),times,255);
        flask::PhysicalSample scalar;const bool accepted=uniModel.sample(boundary,0,scalar);
        require(output.physical_valid==(accepted ? 255u:0u),"angular-rate boundary agrees without widening limits");
    }
    flask::BVP<2,2>::Curve stop;stop.T=1;
    stop.coefficients(0,1)=.3;stop.coefficients(0,2)=-.3;
    double stopTimes[8]{.5,.1,.2,.3,.4,.5,.6,.7};
    const auto zero=flask::evaluateBatch(uniContext,flask::packCurve<2,2>(stop),stopTimes,1);
    require(zero.physical_valid==0,"SIMD zero speed is rejected");
    flask::Validator<2,2> stopValidator(uniModel);
    require(!stopValidator.edge(stop) && stopValidator.stats.reasons[static_cast<std::size_t>(rrtpff::FeasibilityFailure::SpeedZero)]>0,
            "interval singularity rejection keeps its SpeedZero reason");
    auto quad=fixture<4,2>(flask::Robot::Quad3D);flask::BVP<4,2>::Curve freefall;freefall.T=1;
    freefall.coefficients(2,2)=-quad.quad3.gravity/2;
    require(flask::evaluateBatch(flask::BatchContext(quad),flask::packCurve<4,2>(freefall),ht,255).physical_valid==0,
            "SIMD zero-thrust singularity rejected");
});}
