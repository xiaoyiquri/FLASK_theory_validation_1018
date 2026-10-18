#include "flask/io.h"
namespace flask {
int runUnicycle(const Scene &s,const std::filesystem::path &out) {
    if(s.n==2 && s.r==2) return runScene<2,2>(s,out);
    if(s.n==2 && s.r==3) return runScene<2,3>(s,out);
    throw std::invalid_argument("Unicycle CLI requires n=2, r=2 or 3");
}}
