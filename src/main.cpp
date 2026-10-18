#include "flask/io.h"
#include <iostream>
int main(int argc,char **argv) {
    try {
        if(argc<3 || argc>6) { std::cerr<<"Usage: flask_plan scenario.yaml result.yaml [seed [budget_seconds [scalar|simd]]]\n"; return 1; }
        auto scene=flask::loadScene(argv[1]);
        if(argc>=4) {
            const std::string input=argv[3]; std::size_t used=0;
            const auto seed=std::stoull(input,&used);
            if(used!=input.size() || input.empty() || input.front()=='-' || seed>UINT32_MAX)
                throw std::invalid_argument("Seed must be a uint32 integer");
            scene.planner.seed=static_cast<std::uint32_t>(seed);
        }
        if(argc>=5) {
            const std::string input=argv[4]; std::size_t used=0;
            scene.planner.timeout=std::stod(input,&used);
            if(used!=input.size() || !std::isfinite(scene.planner.timeout) || scene.planner.timeout<=0)
                throw std::invalid_argument("Budget must be positive and finite");
        }
        if(argc==6) {
            const std::string backend=argv[5];
            if(backend!="scalar" && backend!="simd") throw std::invalid_argument("Backend must be scalar or simd");
            scene.validation.simd=backend=="simd";
        }
        if(scene.robot_name=="integrator") return flask::runIntegrator(scene,argv[2]);
        if(scene.robot_name=="unicycle") return flask::runUnicycle(scene,argv[2]);
        if(scene.robot_name=="quad2d") return flask::runQuad2(scene,argv[2]);
        return flask::runQuad3(scene,argv[2]);
    } catch(const std::exception &e) { std::cerr<<"ERROR: "<<e.what()<<'\n'; return 1; }
}
