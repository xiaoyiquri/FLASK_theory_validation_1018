#include "flask/batch.h"
#include <immintrin.h>
#include <limits>
namespace flask {
namespace {
struct V {
    __m256d x;
    V():x(_mm256_setzero_pd()) {}
    V(double v):x(_mm256_set1_pd(v)) {}
    explicit V(__m256d v):x(v) {}
    static V load(const double *v) { return V(_mm256_loadu_pd(v)); }
    void store(double *v) const { _mm256_storeu_pd(v,x); }
};
V operator+(V a,V b){return V(_mm256_add_pd(a.x,b.x));}
V operator-(V a,V b){return V(_mm256_sub_pd(a.x,b.x));}
V operator-(V a){return V(0)-a;}
V operator*(V a,V b){return V(_mm256_mul_pd(a.x,b.x));}
V operator/(V a,V b){return V(_mm256_div_pd(a.x,b.x));}
V operator&(V a,V b){return V(_mm256_and_pd(a.x,b.x));}
V operator<=(V a,V b){return V(_mm256_cmp_pd(a.x,b.x,_CMP_LE_OQ));}
V operator<(V a,V b){return V(_mm256_cmp_pd(a.x,b.x,_CMP_LT_OQ));}
V operator>(V a,V b){return b<a;}
V operator>=(V a,V b){return b<=a;}
V abs(V a){return V(_mm256_andnot_pd(_mm256_set1_pd(-0.),a.x));}
V sqrt(V a){return V(_mm256_sqrt_pd(a.x));}
V finite(V a){return abs(a)<=V(std::numeric_limits<double>::max());}
unsigned bits(V a){return unsigned(_mm256_movemask_pd(a.x));}
V select(V mask,V yes,V no){return V(_mm256_blendv_pd(no.x,yes.x,mask.x));}
struct Vec { V x,y,z; };
Vec operator+(Vec a,Vec b){return {a.x+b.x,a.y+b.y,a.z+b.z};}
Vec operator-(Vec a,Vec b){return {a.x-b.x,a.y-b.y,a.z-b.z};}
Vec operator*(Vec a,V s){return {a.x*s,a.y*s,a.z*s};}
Vec operator/(Vec a,V s){return {a.x/s,a.y/s,a.z/s};}
V dot(Vec a,Vec b){return (a.x*b.x+a.y*b.y)+a.z*b.z;}
Vec cross(Vec a,Vec b){return {a.y*b.z-a.z*b.y,a.z*b.x-a.x*b.z,a.x*b.y-a.y*b.x};}
V norm(Vec a){return sqrt(dot(a,a));}
V finite(Vec a){return finite(a.x)&finite(a.y)&finite(a.z);}
// High-order double Taylor on [-pi/2,pi/2]. Large arguments use libm
// range reduction for accuracy; six-scene yaw is fixed at zero.
void sincos(V angle,V &s,V &c) {
    constexpr double pi=3.141592653589793238462643383279502884;
    double raw[4]; angle.store(raw);
    bool small=true; for(double a:raw) small=small && std::isfinite(a) && std::abs(a)<=pi;
    if(!small) { double ss[4],cc[4]; for(int i=0;i<4;++i) { ss[i]=std::sin(raw[i]); cc[i]=std::cos(raw[i]); }
        s=V::load(ss); c=V::load(cc); return; }
    V a=abs(angle),q=select(a>V(pi/2),V(pi)-a,a),z=q*q;
    V ps(1./355687428096000.),pc(1./20922789888000.);
    // sin: terms through x^17; cos: terms through x^16.
    const double st[]={-1./1307674368000.,1./6227020800.,-1./39916800.,1./362880.,-1./5040.,1./120.,-1./6.,1};
    const double ct[]={-1./87178291200.,1./479001600.,-1./3628800.,1./40320.,-1./720.,1./24.,-.5,1};
    for(double k:st) ps=ps*z+V(k);
    for(double k:ct) pc=pc*z+V(k);
    s=select(angle<V(0),-(q*ps),q*ps);
    c=select(a>V(pi/2),-pc,pc);
}
struct Derivatives { Vec u,d,dd; };
Derivatives normalized(Vec v,Vec d,Vec dd,V &valid,double epsilon) {
    V length=norm(v); valid=valid&finite(length)&(length>V(epsilon));
    Vec u=v/length; V first=dot(u,d);
    Vec ud=(d-u*first)/length;
    Vec udd=(dd-u*dot(u,dd))/length-u*dot(ud,ud)-ud*(V(2)*first/length);
    valid=valid&finite(u)&finite(ud)&finite(udd); return {u,ud,udd};
}
using Clock=std::chrono::steady_clock;
double seconds(Clock::time_point start){return std::chrono::duration<double>(Clock::now()-start).count();}
}
BatchResult evaluateBatchAVX2(const BatchContext &ctx,const PackedCurve &curve,const double times[8],
                             unsigned active,bool profile) {
    BatchResult out; const auto &cfg=ctx.model;
    const auto begin=profile ? Clock::now():Clock::time_point{};
    for(int offset=0;offset<8;offset+=4) {
        double safe_times[4]{};
        for(int i=0;i<4;++i) if(active&(1u<<(offset+i))) safe_times[i]=times[offset+i];
        V t=V::load(safe_times); if(curve.normalized) t=t/V(curve.T);
        for(int d=0;d<5;++d) for(int a=0;a<curve.n;++a) {
            V value;
            for(int k=curve.degree-d;k>=0;--k) value=value*t+V(curve.coefficients[d][a][k]);
            value.store(out.jet[d][a]+offset);
        }
    }
    if(profile) out.evaluation_seconds=seconds(begin);
    const auto physical_begin=profile ? Clock::now():Clock::time_point{};
    for(int offset=0;offset<8;offset+=4) {
        unsigned valid=(active>>offset)&15u;
        const auto reject=[&](V condition,rrtpff::FeasibilityFailure reason) {
            const unsigned failed=bits(condition)&valid;
            out.failures[static_cast<std::size_t>(reason)]|=failed<<offset; valid&=~failed;
        };
        const auto keep=[&](V condition,rrtpff::FeasibilityFailure reason) {
            const unsigned failed=(~bits(condition))&valid;
            out.failures[static_cast<std::size_t>(reason)]|=failed<<offset; valid&=~failed;
        };
        const auto jet=[&](int d,int a){return V::load(out.jet[d][a]+offset);};
        const auto vec=[&](int d){return Vec{jet(d,0),jet(d,1),curve.n>=3 ? jet(d,2):V(0)};};
        using F=rrtpff::FeasibilityFailure;
        for(int d=0;d<5;++d) for(int a=0;a<curve.n;++a) keep(finite(jet(d,a)),F::Numeric);
        const int count=cfg.constrain_sampling_bounds || cfg.robot==Robot::Integrator ? curve.n*curve.r:cfg.spatial_dim;
        for(int k=0;k<count;++k) {
            const V inside=(jet(k/curve.n,k%curve.n)>=V(cfg.lower(k)-1e-9)) &
                           (jet(k/curve.n,k%curve.n)<=V(cfg.upper(k)+1e-9));
            const unsigned failed=(~bits(inside))&valid;
            out.flat_bound_rejections|=failed<<offset;valid&=~failed;
        }
        if(!ctx.parameters_valid) keep(V(0)>V(1),F::Numeric);
        V controls[4],omega[3]; Vec orient[3];
        if(cfg.robot==Robot::Unicycle) {
            const auto &c=cfg.unicycle; V vx=jet(1,0),vy=jet(1,1),squared=vx*vx+vy*vy;
            V speed=sqrt(squared),w=(vx*jet(2,1)-vy*jet(2,0))/squared;
            const double initial=std::hypot(curve.coefficients[1][0][0],curve.coefficients[1][1][0]);
            rrtpff::UnicycleTrajectoryMetadata metadata;
            if(!rrtpff::UnicycleDF::makeMetadata(cfg.unicycle_branch*initial,metadata,c)) keep(V(0)>V(1),F::Branch);
            reject(speed<=V(c.velocity_epsilon),F::SpeedZero);
            keep(finite(speed)&finite(w)&(squared>V(c.velocity_epsilon*c.velocity_epsilon)),F::Numeric);
            const double lo=cfg.unicycle_branch<0 ? std::max(0.,-c.velocity_upper):std::max(0.,c.velocity_lower);
            const double hi=std::min(c.max_velocity,cfg.unicycle_branch<0 ? -c.velocity_lower:c.velocity_upper);
            reject(speed<V(lo-1e-10),F::SpeedMin); reject(speed>V(hi+1e-10),F::SpeedMax);
            reject(w<V(std::max(-c.max_omega,c.omega_lower)-1e-10),F::Omega);
            reject(w>V(std::min(c.max_omega,c.omega_upper)+1e-10),F::Omega);
            controls[0]=V(cfg.unicycle_branch)*speed; controls[1]=w; omega[2]=w;
            orient[0]={V(cfg.unicycle_branch)*vx/speed,V(cfg.unicycle_branch)*vy/speed,V(0)};
        } else if(cfg.robot==Robot::Quad2D) {
            const auto &c=cfg.quad2; V ax=jet(2,0),az=jet(2,1)+V(c.gravity);
            V jx=jet(3,0),jz=jet(3,1),sx=jet(4,0),sz=jet(4,1);
            V den=ax*ax+az*az,f=sqrt(den),num=ax*jz-az*jx;
            V w=num/den,alpha=((ax*sz-az*sx)*den-num*(V(2)*(ax*jx+az*jz)))/(den*den);
            reject(f<=V(c.singular_epsilon),F::Singularity);
            reject(norm(vec(1))>V(c.max_velocity+1e-9),F::SpeedMax);
            reject(norm(vec(2))>V(c.max_acceleration+1e-9),F::Acceleration);
            reject(norm(vec(3))>V(c.max_jerk+1e-9),F::Jerk);
            reject(V(c.mass)*f>V(c.max_thrust+1e-9),F::Thrust);
            keep(finite(w)&(den>V(c.singular_epsilon*c.singular_epsilon)),F::Singularity);
            reject(abs(w)>V(c.max_omega+1e-9),F::Omega); keep(finite(alpha),F::Numeric);
            reject(abs(alpha)>V(c.max_angular_acceleration+1e-9),F::AngularAcceleration);
            V thrust=V(c.mass)*f,difference=V(c.inertia/c.arm_length)*alpha;
            controls[0]=(thrust+difference)/V(c.mass*c.gravity);
            controls[1]=(thrust-difference)/V(c.mass*c.gravity);
            for(int k=0;k<2;++k) keep(finite(controls[k])&(controls[k]>=V(-1e-9))&(controls[k]<=V(c.max_motor+1e-9)),F::Motor);
            omega[2]=w; orient[0]={az/f,-ax/f,V(0)};
        } else if(cfg.robot==Robot::Quad3D) {
            const auto &c=cfg.quad3; Vec force=vec(2)+Vec{V(0),V(0),V(c.gravity)};
            reject(norm(force)<=V(c.singular_epsilon),F::Singularity);
            reject(norm(vec(1))>V(c.max_velocity+1e-9),F::SpeedMax);
            reject(norm(vec(2))>V(c.max_acceleration+1e-9),F::Acceleration);
            reject(norm(vec(3))>V(c.max_jerk+1e-9),F::Jerk);
            reject(norm(force)>V(c.gravity*c.max_motor+1e-9),F::Thrust);
            V good(1); good=good>V(0);
            auto b3=normalized(force,vec(3),vec(4),good,c.singular_epsilon);
            V sy,cy; sincos(jet(0,3),sy,cy); V yr=jet(1,3),ya=jet(2,3);
            Vec h{cy,sy,V(0)},hd{-sy*yr,cy*yr,V(0)},hdd{-cy*yr*yr-sy*ya,-sy*yr*yr+cy*ya,V(0)};
            auto b2=normalized(cross(b3.u,h),cross(b3.d,h)+cross(b3.u,hd),
                               cross(b3.dd,h)+cross(b3.d,hd)*V(2)+cross(b3.u,hdd),good,c.singular_epsilon);
            Derivatives b1{cross(b2.u,b3.u),cross(b2.d,b3.u)+cross(b2.u,b3.d),
                cross(b2.dd,b3.u)+cross(b2.d,b3.d)*V(2)+cross(b2.u,b3.dd)};
            keep(good&finite(b1.u)&finite(b1.d)&finite(b1.dd)&(abs(dot(b1.u,cross(b2.u,b3.u))-V(1))<V(1e-6)),F::Singularity);
            Derivatives b[]={b1,b2,b3};
            auto skew=[&](int i,int j,bool derivative) {
                if(!derivative) return (dot(b[i].u,b[j].d)-dot(b[j].u,b[i].d))*V(.5);
                return ((dot(b[i].d,b[j].d)+dot(b[i].u,b[j].dd))-
                        (dot(b[j].d,b[i].d)+dot(b[j].u,b[i].dd)))*V(.5);
            };
            Vec w{skew(2,1,false),skew(0,2,false),skew(1,0,false)};
            Vec alpha{skew(2,1,true),skew(0,2,true),skew(1,0,true)};
            reject(norm(w)>V(c.max_omega+1e-9),F::Omega);
            reject(norm(alpha)>V(c.max_angular_acceleration+1e-9),F::AngularAcceleration);
            Vec iw{w.x*V(c.inertia.x()),w.y*V(c.inertia.y()),w.z*V(c.inertia.z())};
            Vec torque=Vec{alpha.x*V(c.inertia.x()),alpha.y*V(c.inertia.y()),alpha.z*V(c.inertia.z())}+cross(w,iw);
            V wrench[]={V(c.mass)*norm(force),torque.x,torque.y,torque.z};
            for(int k=0;k<4;++k) {
                for(int j=0;j<4;++j) controls[k]=controls[k]+V(ctx.inverse(k,j))*wrench[j];
                const double tol=c.control_map ? 1e-9:0;
                keep(finite(controls[k])&(controls[k]>=V(ctx.control_lower(k)-tol))&
                     (controls[k]<=V(ctx.control_upper(k)+tol)),F::Motor);
            }
            omega[0]=w.x;omega[1]=w.y;omega[2]=w.z;orient[0]=b1.u;orient[1]=b2.u;orient[2]=b3.u;
        } else {
            for(int k=0;k<curve.n;++k) controls[k]=jet(curve.r,k);
        }
        for(int k=0;k<4;++k) controls[k].store(out.control[k]+offset);
        for(int k=0;k<3;++k) {
            omega[k].store(out.omega[k]+offset);
            orient[k].x.store(out.orientation[3*k]+offset);orient[k].y.store(out.orientation[3*k+1]+offset);
            orient[k].z.store(out.orientation[3*k+2]+offset);
        }
        out.physical_valid|=valid<<offset;
    }
    if(profile) out.physical_seconds=seconds(physical_begin);
    const auto collision_begin=profile ? Clock::now():Clock::time_point{};
    if(out.physical_valid==active) {
        for(std::size_t i=0;i<ctx.obstacle_x.size() && !out.collision;++i) for(int offset=0;offset<8;offset+=4) {
            V x=V::load(out.jet[0][0]+offset)-V(ctx.obstacle_x[i]);
            V y=(cfg.spatial_dim>=2 ? V::load(out.jet[0][1]+offset):V(0))-V(ctx.obstacle_y[i]);
            V z=(cfg.spatial_dim==3 ? V::load(out.jet[0][2]+offset):V(0))-V(ctx.obstacle_z[i]);
            if(bits((x*x+y*y)+z*z<=V(ctx.radius_squared[i]))&((active>>offset)&15u)) { out.collision=true; break; }
        }
    }
    if(profile) out.collision_seconds=seconds(collision_begin);
    return out;
}
} // namespace flask
