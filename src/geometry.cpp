#include "flask/geometry.h"
#include <cmath>
#include <stdexcept>
#if defined(_MSC_VER) && (defined(_M_X64) || defined(_M_IX86))
#include <intrin.h>
#elif (defined(__GNUC__) || defined(__clang__)) && (defined(__x86_64__) || defined(__i386__))
#include <cpuid.h>
#endif
namespace flask {
#ifdef FLASK_HAS_AVX2
unsigned collisionMaskAVX2(const PositionBatch &,const std::vector<Sphere> &,double);
#endif
static bool detectAVX2() {
#if defined(FLASK_HAS_AVX2) && defined(_MSC_VER)
    int regs[4]; __cpuid(regs,0); if(regs[0]<7) return false;
    __cpuid(regs,1);
    if(!(regs[2]&(1<<27)) || !(regs[2]&(1<<28)) || (_xgetbv(0)&6)!=6) return false;
    __cpuidex(regs,7,0); return (regs[1]&(1<<5))!=0;
#elif defined(FLASK_HAS_AVX2) && (defined(__x86_64__) || defined(__i386__))
    // Direct CPUID also works with compiler-rt Windows toolchains that do
    // not supply GCC's __cpu_model runtime. Check OS YMM state first.
    if(__get_cpuid_max(0,nullptr)<7) return false;
    unsigned a,b,c,d; __cpuid_count(1,0,a,b,c,d);
    if(!(c&(1u<<27)) || !(c&(1u<<28))) return false;
    unsigned low,high;
    __asm__ volatile("xgetbv" : "=a"(low),"=d"(high) : "c"(0));
    if((low&6u)!=6u) return false;
    __cpuid_count(7,0,a,b,c,d); return (b&(1u<<5))!=0;
#else
    return false;
#endif
}
bool avx2Available() { static const bool supported=detectAVX2(); return supported; }
unsigned collisionMask(const PositionBatch &batch,const std::vector<Sphere> &obstacles,
                       double robotRadius,bool simd) {
    if(!std::isfinite(robotRadius)||robotRadius<0) throw std::invalid_argument("Invalid robot radius");
    unsigned mask=0;
    for(int i=0;i<8;++i) if(batch.active&(1u<<i))
        if(!batch.position[i].allFinite()) mask|=1u<<i;
    for(const auto &s:obstacles)
        if(!s.center.allFinite() || !std::isfinite(s.radius) || s.radius<0)
            throw std::invalid_argument("Invalid sphere");
#ifdef FLASK_HAS_AVX2
    if(simd && avx2Available()) return mask|collisionMaskAVX2(batch,obstacles,robotRadius);
#else
    (void)simd;
#endif
    for(const auto &s:obstacles) for(int i=0;i<8;++i) if(batch.active&(1u<<i)) {
        const double radius=s.radius+robotRadius;
        const Eigen::Vector3d d=batch.position[i]-s.center;
        if((d.x()*d.x()+d.y()*d.y())+d.z()*d.z()<=radius*radius) mask|=1u<<i;
    }
    return mask;
}
} // namespace flask
