#include "flask/geometry.h"
#include <immintrin.h>
namespace flask {
// Two double4 registers retain the scalar model's precision while checking
// eight configurations. AVX flags are confined to this translation unit.
unsigned collisionMaskAVX2(const PositionBatch &b,const std::vector<Sphere> &obstacles,double robotRadius) {
    unsigned result=0;
    for(int offset=0;offset<8;offset+=4) {
        const auto x=_mm256_set_pd(b.position[offset+3].x(),b.position[offset+2].x(),b.position[offset+1].x(),b.position[offset].x());
        const auto y=_mm256_set_pd(b.position[offset+3].y(),b.position[offset+2].y(),b.position[offset+1].y(),b.position[offset].y());
        const auto z=_mm256_set_pd(b.position[offset+3].z(),b.position[offset+2].z(),b.position[offset+1].z(),b.position[offset].z());
        for(const auto &s:obstacles) {
            const auto dx=_mm256_sub_pd(x,_mm256_set1_pd(s.center.x()));
            const auto dy=_mm256_sub_pd(y,_mm256_set1_pd(s.center.y()));
            const auto dz=_mm256_sub_pd(z,_mm256_set1_pd(s.center.z()));
            const auto sum=_mm256_add_pd(_mm256_add_pd(_mm256_mul_pd(dx,dx),_mm256_mul_pd(dy,dy)),_mm256_mul_pd(dz,dz));
            const double radius=s.radius+robotRadius;
            result|=unsigned(_mm256_movemask_pd(_mm256_cmp_pd(sum,_mm256_set1_pd(radius*radius),_CMP_LE_OQ)))<<offset;
        }
    }
    return result&b.active;
}
} // namespace flask
