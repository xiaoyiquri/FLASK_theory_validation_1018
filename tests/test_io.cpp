#include "flask/io.h"
#include "theory_test_support.h"
#include <fstream>
using namespace theory;
int main() { return run([] {
    const auto file=std::filesystem::path("io_fixture.yaml");
    { std::ofstream out(file); out<<"robot: integrator\nn: 1\nr: 1\nrobot_radius: 0\nstart: [0]\ngoal: [1]\nbounds: {lower: [-2], upper: [2]}\nplanner: {type: rrtconnect, seed: 9}\n"; }
    const auto scene=flask::loadScene(file);
    require(scene.n==1 && scene.r==1 && scene.planner.seed==9,"YAML options loaded");
    const int code=flask::runScene<1,1>(scene,"io_result.yaml");
    require(code==0,"end-to-end serialization");
    const auto result=YAML::LoadFile("io_result.yaml");
    require(result["success"].as<bool>() && result["segments"].size()==1,"trajectory has exact coefficients");
    require(!result["asymptotic_optimality_verified"].as<bool>(),"no unsupported AO claim");
    require(result["samples"].size()>2,"physical samples exported");
    require(result["result_version"].as<int>()==2 && result["incumbents"].size()==1,"versioned event history exported");
    const auto first=YAML::LoadFile("first_solution/io_result.yaml"),final=YAML::LoadFile("final_solution/io_result.yaml");
    relative(first["cost"].as<double>(),result["first_solution_cost"].as<double>(),1e-12,"first cost belongs to saved first trajectory");
    relative(first["trajectory_length_m"].as<double>(),result["first_solution_length_m"].as<double>(),1e-12,"first length belongs to saved first trajectory");
    relative(final["cost"].as<double>(),result["cost"].as<double>(),1e-12,"final trajectory has reported objective");
    require(first["available_seconds"].as<double>()==result["search_seconds"].as<double>(),"first time and path share a discovery event");
    require(first["segments"][0]["normalized_time"].IsDefined(),"coefficient time convention explicit");
    relative(result["trajectory_length_m"].as<double>(),1,1e-10,"metric length is separate from BVP cost");
    const auto native=YAML::LoadFile("io_result.dynobench.yaml");
    require(native["result"][0]["states"].size()==native["result"][0]["actions"].size()+1,
            "DynoBench viewer states/actions layout");
    require(!native["dynobench_checker_verified"].as<bool>(),"resampling must not claim dynamics certification");
    require(native["result"][0]["times"].size()==native["result"][0]["states"].size(),"native checker receives actual last-step timestamps");
    auto invalid=scene; invalid.model.obstacles.push_back({Eigen::Vector3d::Zero(),10});
    require(flask::runScene<1,1>(invalid,"io_failed.yaml")==2,"failed run remains failed");
    const auto failed=YAML::LoadFile("io_failed.yaml");
    require(failed["first_solution"].IsNull() && failed["first_solution_cost"].IsNull() && failed["incumbents"].size()==0,"failure cannot masquerade as zero-cost first solution");
    { std::ofstream out(file); out<<"robot: integrator\nn: 1\nr: 1\nrobot_radius: 0\nstart: [0,1]\ngoal: [1]\nbounds: {lower: [-2], upper: [2]}\n"; }
    bool rejected=false; try { flask::loadScene(file); } catch(const std::exception &) { rejected=true; }
    require(rejected,"wrong input vector dimension rejected");
    { std::ofstream out("io_environment.yaml"); out<<
        "environment: {min: [-2,-2], max: [2,2], obstacles: [{type: box, center: [0,1], size: [0.2,0.2]}]}\n"
        "robots: [{type: unicycle1_v0, start: [0,0,0], goal: [1,0,0]}]\n"; }
    { std::ofstream out("io_model.yaml"); out<<
        "dynamics: unicycle1\nmax_vel: 1\nmin_vel: 0\nmax_angular_vel: 1.5\nmin_angular_vel: -1.5\n"; }
    { std::ofstream out(file); out<<
        "robot: unicycle\nn: 2\nr: 2\nrobot_radius: 0.1\n"
        "bounds: {lower: [-2,-2,-1,-1], upper: [2,2,1,1]}\n"
        "dynobench: {environment: io_environment.yaml, model: io_model.yaml}\n"
        "lifting: {start: {speed: 0.2}, goal: {speed: 0.2}}\n"
        "planner: {type: rrtconnect}\n"; }
    const auto imported=flask::loadScene(file);
    require(imported.start.size()==4 && imported.start(2)==.2,"explicit DynoBench boundary lifting");
    require(imported.model.physical_goal.size()==3 && !imported.model.obstacles.empty(),"DynoBench physical endpoints and box cover");
    require(YAML::Load(imported.configuration)["dynobench_model_snapshot"]["max_vel"].as<double>()==1,
            "external model snapshot is recorded");
    require(flask::runScene<2,2>(imported,"io_dynobench_result.yaml")==0,"DynoBench data-only import plans and exports");
    auto wrongBranch=YAML::Load(imported.configuration);
    wrongBranch["lifting"]["start"]["speed"]=-.2;
    { std::ofstream out(file); out<<YAML::Dump(wrongBranch); }
    rejected=false; try { flask::loadScene(file); } catch(const std::exception &) { rejected=true; }
    require(rejected,"inconsistent speed/branch rejected at input");
}); }
