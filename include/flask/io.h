#pragma once
#include "flask/result_io.h"
#include <filesystem>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <yaml-cpp/yaml.h>
namespace flask {
struct Scene {
    int n=2,r=2;
    std::string robot_name="integrator",planner_name="rrtconnect",source,configuration;
    Eigen::VectorXd start,goal;
    Eigen::MatrixXd weight;
    double rho=1,output_step=.01;
    ModelConfig model;
    PlannerConfig planner;
    ValidationConfig validation;
};
Scene loadScene(const std::filesystem::path &);
void emitVector(YAML::Emitter &,const Eigen::VectorXd &);
template<int N,int R> int runScene(const Scene &scene,const std::filesystem::path &output) {
    using State=typename BVP<N,R>::State;
    if(scene.start.size()!=N*R || scene.goal.size()!=N*R) throw std::invalid_argument("Endpoint dimension mismatch");
    State start,goal; start.values=scene.start; goal.values=scene.goal;
    Model<N,R> model(scene.model); BVP<N,R> bvp(scene.weight,scene.rho);
    Validator<N,R> validator(model,scene.validation);
    if(scene.planner_name!="rrtconnect") throw std::invalid_argument("Only FLASK-RRTConnect is implemented");
    RRTConnect<N,R> planner(model,bvp,validator,scene.planner);
    const auto plan=planner.solve(start,goal);
    // All length integrals, diagnostics and serialization are outside the
    // measured search/shortcut clock. History times are actual observations.
    YAML::Node common,configuration=YAML::Load(scene.configuration);
    configuration["planner"]["seed"]=scene.planner.seed;
    configuration["planner"]["timeout"]=scene.planner.timeout;
    configuration["validation"]["simd"]=scene.validation.simd;
    common["result_version"]=2; common["source"]=scene.source;
    common["input_configuration"]=configuration;
    common["robot"]=scene.robot_name; common["planner"]=scene.planner_name;
    common["seed"]=scene.planner.seed; common["n"]=N; common["r"]=R; common["rho"]=scene.rho;
    common["budget_seconds"]=scene.planner.timeout;
    common["planning_time_basis"]="wall elapsed search + Algorithm 4, excludes export and length integration";
    common["numeric_precision"]="float64"; common["sampling_policy"]="flat_uniform";
    common["validation_backend"]=validator.vectorized() ? "avx2_double4x2_full":"scalar_float64";
    common["asymptotic_optimality_verified"]=false; common["continuous_control_certificate"]=false;
    common["continuous_sphere_check"]=scene.validation.continuous_sphere_collision;
    common["planning_geometry"]="single sphere robot; conservative sphere cover of obstacles";
    common["physical_continuity_enforced"]=scene.validation.enforce_physical_continuity;
    common["physical_endpoints_enforced"]=scene.validation.enforce_physical_endpoints;
    common["dynobench_checker_verified"]=false;
    common["reduced_order_result"]=(scene.model.robot==Robot::Quad2D || scene.model.robot==Robot::Quad3D)&&R==2;
    common["avx2_available"]=avx2Available(); common["sampling_step"]=scene.validation.step;
    common["goal_region_radius"]=0; common["flat_start"]=yamlVector(start.values); common["flat_goal"]=yamlVector(goal.values);
    YAML::Node history(YAML::NodeType::Sequence),first,final,document;
    double length=0;
    for(std::size_t i=0;i<plan.history.size();++i) {
        const auto &event=plan.history[i];
        YAML::Node snapshot=trajectoryNode(common,model,event,scene.validation.step);
        std::filesystem::path relative;
        if(i==0) relative=std::filesystem::path("first_solution")/output.filename();
        else relative=std::filesystem::path("incumbents")/(output.stem().string()+"_"+std::to_string(i)+".yaml");
        snapshot["first_solution_seconds"]=plan.search_seconds;
        writeYaml(output.parent_path()/relative,snapshot);
        YAML::Node metric; metric["index"]=i; metric["stage"]=event.stage;
        metric["available_seconds"]=event.seconds; metric["cost"]=snapshot["cost"].as<double>();
        metric["trajectory_length_m"]=snapshot["trajectory_length_m"].as<double>();
        metric["duration"]=snapshot["duration"].as<double>(); metric["trajectory_file"]=relative.generic_string();
        history.push_back(metric);
        if(i==0) {
            first=YAML::Clone(metric);
            auto native=output.parent_path()/relative; native.replace_extension(".dynobench.yaml");
            writeNative<N,R>(native,model,event.curves,scene.output_step);
        }
        if(i+1==plan.history.size()) {
            final=YAML::Clone(metric); document=YAML::Clone(snapshot);
            const auto relativeFinal=std::filesystem::path("final_solution")/output.filename();
            final["trajectory_file"]=relativeFinal.generic_string();
            snapshot["stage"]="final_solution"; writeYaml(output.parent_path()/relativeFinal,snapshot);
            auto native=output.parent_path()/relativeFinal; native.replace_extension(".dynobench.yaml");
            writeNative<N,R>(native,model,event.curves,scene.output_step);
            auto legacy=output; legacy.replace_extension(".dynobench.yaml");
            writeNative<N,R>(legacy,model,event.curves,scene.output_step);
            length=metric["trajectory_length_m"].as<double>();
        }
    }
    if(!plan.success) {
        document=YAML::Clone(common); document["cost"]=YAML::Node(); document["duration"]=YAML::Node();
        document["trajectory_length_m"]=YAML::Node();
        document["segments"]=YAML::Node(YAML::NodeType::Sequence);
        document["samples"]=YAML::Node(YAML::NodeType::Sequence);
    }
    document["success"]=plan.success; document["reason"]=plan.reason;
    document["iterations"]=plan.iterations; document["nodes"]=plan.nodes;
    document["bvp_queries"]=plan.bvp_queries; document["shortcuts"]=plan.shortcuts;
    document["extension_attempts"]=plan.extension_attempts; document["valid_extensions"]=plan.valid_extensions;
    document["valid_extension_rate"]=plan.extension_attempts ? double(plan.valid_extensions)/plan.extension_attempts:0.;
    document["planning_seconds"]=plan.seconds; document["search_seconds"]=plan.search_seconds;
    document["postprocessing_seconds"]=std::max(0.,plan.seconds-plan.search_seconds);
    document["first_solution"]=first; document["final_solution"]=final; document["incumbents"]=history;
    document["first_solution_seconds"]=!plan.history.empty() ? YAML::Node(plan.search_seconds):YAML::Node();
    document["first_solution_cost"]=!plan.history.empty() ? YAML::Node(first["cost"].as<double>()):YAML::Node();
    document["first_solution_length_m"]=!plan.history.empty() ? YAML::Node(first["trajectory_length_m"].as<double>()):YAML::Node();
    document["cost_before_shortcut"]=!plan.history.empty() ? YAML::Node(first["cost"].as<double>()):YAML::Node();
    document["terminated_early"]=plan.seconds<scene.planner.timeout;
    document["cost_time_semantics"]="first solution and accepted shortcuts only; hold incumbent after termination, no restarts";
    auto &stats=validator.stats; YAML::Node validation;
    validation["edges"]=stats.edges; validation["samples"]=stats.samples; validation["batches"]=stats.batches;
    validation["collision_rejections"]=stats.collision_rejections; validation["physical_rejections"]=stats.physical_rejections;
    validation["certificate_rejections"]=stats.certificate_rejections; validation["interval_rejections"]=stats.interval_rejections;
    validation["simd_batches"]=stats.simd_batches;
    validation["sample_budget_rejections"]=stats.sample_budget_rejections;
    validation["flat_bound_rejections"]=stats.flat_bound_rejections;
    for(std::size_t k=1;k<stats.reasons.size();++k) validation["failure_counts"][rrtpff::failureNames[k]]=stats.reasons[k];
    validation["failure_counts"]["Collision"]=stats.collision_rejections;
    validation["failure_counts"]["FlatBounds"]=stats.flat_bound_rejections;
    validation["failure_counts"]["SampleBudget"]=stats.sample_budget_rejections;
    document["validation"]=validation;
    YAML::Node profiling; profiling["enabled"]=scene.planner.profile || scene.validation.profile;
    profiling["nearest_seconds"]=plan.nearest_seconds; profiling["bvp_seconds"]=plan.bvp_seconds;
    profiling["interval_seconds"]=stats.interval_seconds; profiling["jet_seconds"]=stats.evaluation_seconds;
    profiling["physical_seconds"]=stats.physical_seconds; profiling["collision_seconds"]=stats.collision_seconds;
    profiling["certificate_seconds"]=stats.certificate_seconds; document["profiling"]=profiling;
    writeYaml(output,document);
    std::cout<<std::setprecision(17)<<(plan.success ? "SUCCESS ":"FAILED ")<<plan.reason<<" nodes="<<plan.nodes
             <<" cost="<<plan.cost()<<" seconds="<<plan.seconds<<" length="<<length
             <<" search_seconds="<<plan.search_seconds<<'\n';
    return plan.success ? 0:2;
}
int runIntegrator(const Scene &,const std::filesystem::path &);
int runUnicycle(const Scene &,const std::filesystem::path &);
int runQuad2(const Scene &,const std::filesystem::path &);
int runQuad3(const Scene &,const std::filesystem::path &);
} // namespace flask
