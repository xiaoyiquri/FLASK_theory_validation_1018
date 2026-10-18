#include "flask/io.h"
namespace flask {
int runQuad3(const Scene &s,const std::filesystem::path &out) {
    if(s.n==4 && s.r==2) return runScene<4,2>(s,out);
    if(s.n==4 && s.r==4) return runScene<4,4>(s,out);
    if(s.n==4 && s.r==5) return runScene<4,5>(s,out);
    throw std::invalid_argument("Quad3D CLI requires n=4, r=2,4 or 5");
}}
