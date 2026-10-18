#include "flask/io.h"
namespace flask {
int runQuad2(const Scene &s,const std::filesystem::path &out) {
    if(s.n==2 && s.r==2) return runScene<2,2>(s,out);
    if(s.n==2 && s.r==4) return runScene<2,4>(s,out);
    if(s.n==2 && s.r==5) return runScene<2,5>(s,out);
    throw std::invalid_argument("Quad2D CLI requires n=2, r=2,4 or 5");
}}
