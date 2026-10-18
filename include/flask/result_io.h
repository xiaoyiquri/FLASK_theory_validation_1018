#pragma once
#include "flask/planner.h"
#include "flask/diagnostics.h"
#include <filesystem>
#include <fstream>
#include <yaml-cpp/yaml.h>

namespace flask {
template<class Derived> YAML::Node yamlVector(const Eigen::MatrixBase<Derived> &v) {
    YAML::Node out(YAML::NodeType::Sequence);
    for(Eigen::Index i=0;i<v.size();++i) out.push_back(double(v(i)));
    out.SetStyle(YAML::EmitterStyle::Flow); return out;
}
inline void writeYaml(const std::filesystem::path &path,const YAML::Node &node) {
    if(!path.parent_path().empty()) std::filesystem::create_directories(path.parent_path());
    YAML::Emitter out; out.SetDoublePrecision(17); out<<node;
    if(!out.good()) throw std::runtime_error(out.GetLastError());
    std::ofstream file(path); file<<out.c_str()<<'\n'; file.close();
    if(!file) throw std::runtime_error("Result write failed: "+path.string());
}
inline YAML::Node boundaryNode(const BoundaryDiagnostics &b) {
    YAML::Node out; out["available"]=b.available;
    if(b.available) {
        out["position_error_m"]=b.position;
        // Unicycle1 speed/rate are controls, absent from its physical state.
        out["velocity_error"]=b.motion_state_available ? YAML::Node(b.velocity):YAML::Node();
        out["attitude_error_rad"]=b.attitude;
        out["angular_velocity_error"]=b.motion_state_available ? YAML::Node(b.angular_velocity):YAML::Node();
        out["full_state_error"]=b.full_state;
    }
    return out;
}
inline YAML::Node diagnosticsNode(const PathDiagnostics &d) {
    YAML::Node out; out["available"]=d.available;
    out["max_position_jump"]=d.position; out["max_velocity_jump"]=d.velocity;
    out["max_acceleration_jump"]=d.acceleration; out["max_jerk_jump"]=d.jerk;
    out["max_attitude_jump_rad"]=d.attitude; out["max_angular_velocity_jump"]=d.angular_velocity;
    out["max_physical_junction_error"]=d.full_state;
    out["start"]=boundaryNode(d.start); out["goal"]=boundaryNode(d.goal);
    out["diagnostic_only"]=true; return out;
}
template<int N,int R> YAML::Node trajectoryNode(const YAML::Node &common,const Model<N,R> &model,
        const Incumbent<N,R> &event,double step) {
    YAML::Node out=YAML::Clone(common);
    Plan<N,R> view; view.curves=event.curves;
    out["success"]=true; out["stage"]=event.stage; out["available_seconds"]=event.seconds;
    out["cost"]=view.cost(); out["duration"]=view.duration();
    out["trajectory_length_m"]=view.length(model.config().spatial_dim);
    const auto d=diagnose(model,event.curves); out["diagnostics"]=diagnosticsNode(d);
    out["physical_start_error"]=d.start.full_state; out["physical_goal_error"]=d.goal.full_state;
    out["max_physical_junction_error"]=d.full_state;
    YAML::Node segments(YAML::NodeType::Sequence),samples(YAML::NodeType::Sequence);
    double elapsed=0;
    for(std::size_t index=0;index<event.curves.size();++index) {
        const auto &curve=event.curves[index]; YAML::Node segment;
        segment["duration"]=curve.T; segment["cost"]=curve.cost; segment["normalized_time"]=curve.normalized;
        for(int axis=0;axis<N;++axis) segment["power_coefficients"].push_back(yamlVector(curve.coefficients.row(axis)));
        segments.push_back(segment);
        const auto intervals=std::size_t(std::max(1.,std::ceil(curve.T/step)));
        if(intervals>1000000) throw std::runtime_error("Output sample budget exceeded");
        // Retain both sides of each join: omitting the right sample would
        // hide reduced-r2 attitude/rate discontinuities in diagnostics/plots.
        for(std::size_t i=0;i<=intervals;++i) {
            const double t=curve.T*(double(i)/intervals); PhysicalSample p;
            if(!model.sample(curve,t,p)) throw std::runtime_error("Invalid physical export sample");
            YAML::Node point; point["time"]=elapsed+t; point["segment"]=index;
            point["state"]=yamlVector(p.state); point["control"]=yamlVector(p.control); samples.push_back(point);
        }
        elapsed+=curve.T;
    }
    out["segments"]=segments; out["samples"]=samples; return out;
}
template<int N,int R> void writeNative(const std::filesystem::path &file,const Model<N,R> &model,
        const std::vector<typename BVP<N,R>::Curve> &curves,double step) {
    if(curves.empty()) return;
    double duration=0; for(const auto &c:curves) duration+=c.T;
    const auto intervals=std::size_t(std::max(1.,std::ceil(duration/step)));
    if(intervals>1000000) throw std::runtime_error("Native sample budget exceeded");
    YAML::Node out,trajectory,states(YAML::NodeType::Sequence),actions(YAML::NodeType::Sequence),times(YAML::NodeType::Sequence);
    std::size_t segment=0; double offset=0;
    for(std::size_t i=0;i<=intervals;++i) {
        const double t=std::min(double(i)*step,duration);
        while(segment+1<curves.size() && t>offset+curves[segment].T) { offset+=curves[segment].T; ++segment; }
        PhysicalSample p;
        if(!model.sample(curves[segment],std::clamp(t-offset,0.,curves[segment].T),p))
            throw std::runtime_error("Invalid native export sample");
        states.push_back(yamlVector(p.state)); times.push_back(t);
        if(i<intervals) actions.push_back(yamlVector(p.control));
    }
    trajectory["states"]=states; trajectory["actions"]=actions; trajectory["times"]=times;
    out["dt"]=step; out["last_step_duration"]=duration-(intervals-1)*step;
    out["dynobench_checker_verified"]=false;
    out["result"].push_back(trajectory); writeYaml(file,out);
}
} // namespace flask
