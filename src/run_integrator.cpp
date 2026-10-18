#include "flask/io.h"
namespace flask {
int runIntegrator(const Scene &s,const std::filesystem::path &out) {
    if(s.n==1 && s.r==1) return runScene<1,1>(s,out);
    if(s.n==2 && s.r==2) return runScene<2,2>(s,out);
    if(s.n==7 && s.r==2) return runScene<7,2>(s,out);
    throw std::invalid_argument("Integrator CLI supports (n,r)=(1,1),(2,2),(7,2); C++ API is templated");
}}
