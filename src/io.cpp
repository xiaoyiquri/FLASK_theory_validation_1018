#include "flask/io.h"
#include "df/paper_r2_df.h"
#include <fstream>

namespace flask {
namespace {
Eigen::VectorXd vector(const YAML::Node &node,int size) {
    if(!node.IsSequence() || node.size()!=std::size_t(size)) throw std::invalid_argument("YAML vector dimension mismatch");
    Eigen::VectorXd out(size);
    for(int k=0;k<size;++k) out(k)=node[k].as<double>();
    if(!out.allFinite()) throw std::invalid_argument("Nonfinite YAML vector");
    return out;
}
template<class T> void option(const YAML::Node &node,const char *key,T &target) {
    if(node && node[key]) target=node[key].as<T>();
}
std::filesystem::path resolve(const std::filesystem::path &base,const YAML::Node &n) {
    std::filesystem::path p(n.as<std::string>()); return p.is_absolute() ? p:base/p;
}
void geometry(const YAML::Node &environment,ModelConfig &model,double cell) {
    const int dim=model.spatial_dim;
    for(const auto &node:environment["obstacles"]) {
        const auto center=vector(node["center"],dim);
        Eigen::Vector3d c=Eigen::Vector3d::Zero(); c.head(dim)=center;
        const auto kind=node["type"].as<std::string>();
        if(kind=="sphere") { model.obstacles.push_back({c,node["radius"].as<double>()}); continue; }
        if(kind!="box" || node["rotation"] || node["quaternion"])
            throw std::invalid_argument("Supported obstacles: spheres or axis-aligned boxes");
        const auto size=vector(node["size"],dim);
        if((size.array()<=0).any() || !std::isfinite(cell)||cell<=0)
            throw std::invalid_argument("Invalid sphere cover cell/box");
        Eigen::Vector3i count=Eigen::Vector3i::Ones(); Eigen::Vector3d step=Eigen::Vector3d::Zero();
        double total=1;
        for(int k=0;k<dim;++k) {
            const double q=std::ceil(size(k)/cell);
            if(q>10000 || total*q>100000) throw std::invalid_argument("Sphere cover exceeds resource budget");
            count(k)=int(q); step(k)=size(k)/q; total*=q;
        }
        // Each cell's circumsphere encloses the original box cell.
        const double radius=std::nextafter(step.norm()/2,INFINITY);
        for(int i=0;i<count.x();++i) for(int j=0;j<count.y();++j) for(int k=0;k<count.z();++k) {
            Eigen::Vector3d point=c;
            const int index[3]{i,j,k};
            for(int axis=0;axis<dim;++axis) point(axis)+=-size(axis)/2+(index[axis]+0.5)*step(axis);
            model.obstacles.push_back({point,radius});
        }
    }
    if(model.obstacles.size()>100000) throw std::invalid_argument("Too many obstacle spheres");
}
void robotParameters(const YAML::Node &node,ModelConfig &model,bool dyno) {
    if(!node) return;
    if(model.robot==Robot::Unicycle) {
        auto &c=model.unicycle;
        option(node,dyno ? "max_vel":"max_velocity",c.max_velocity);
        option(node,dyno ? "max_angular_vel":"max_omega",c.max_omega);
        option(node,dyno ? "min_vel":"velocity_lower",c.velocity_lower);
        option(node,dyno ? "min_angular_vel":"omega_lower",c.omega_lower);
        c.velocity_upper=c.max_velocity; c.omega_upper=c.max_omega;
        option(node,"allow_reverse",c.allow_reverse);
        if(dyno && c.velocity_lower<0) c.allow_reverse=true;
        option(node,"velocity_epsilon",c.velocity_epsilon);
        return;
    }
    if(model.robot==Robot::Integrator) return;
    if(node["drag_against_vel"].as<bool>(false)) throw std::invalid_argument("Drag is not covered by this flat mapping");
    if(dyno && node["g"].as<double>(9.81)!=9.81)
        throw std::invalid_argument("Audited DynoBench dynamics use fixed g=9.81");
    if(model.robot==Robot::Quad2D) {
        auto &c=model.quad2;
        option(node,dyno ? "m":"mass",c.mass); option(node,dyno ? "g":"gravity",c.gravity);
        option(node,dyno ? "I":"inertia",c.inertia); option(node,dyno ? "l":"arm_length",c.arm_length);
        option(node,dyno ? "max_f":"max_motor",c.max_motor);
        option(node,dyno ? "max_vel":"max_velocity",c.max_velocity);
        option(node,dyno ? "max_angular_vel":"max_omega",c.max_omega);
        // DynoBench max_acc/max_angular_acc are heuristic bounds, not physical constraints.
        if(dyno) { c.max_acceleration=INFINITY; c.max_angular_acceleration=INFINITY; }
        else { option(node,"max_acceleration",c.max_acceleration); option(node,"max_angular_acceleration",c.max_angular_acceleration); }
    } else {
        auto &c=model.quad3;
        option(node,dyno ? "m":"mass",c.mass); option(node,dyno ? "g":"gravity",c.gravity);
        option(node,"arm_length",c.arm_length); option(node,dyno ? "t2t":"thrust_to_torque",c.thrust_to_torque);
        const auto inertia=node[dyno ? "J_v":"inertia"];
        if(inertia) c.inertia=vector(inertia,3);
        option(node,dyno ? "max_f":"max_motor",c.max_motor);
        option(node,dyno ? "max_vel":"max_velocity",c.max_velocity);
        option(node,dyno ? "max_angular_vel":"max_omega",c.max_omega);
        if(dyno) {
            c.max_acceleration=INFINITY; c.max_angular_acceleration=INFINITY;
            auto map=std::make_shared<rrtpff::WrenchControlMap>();
            Eigen::Matrix4d allocation=Eigen::Matrix4d::Zero();
            if(node["motor_control"].as<bool>(true)) {
                const double a=.707106781*c.arm_length,k=c.thrust_to_torque;
                allocation<<1,1,1,1,-a,-a,a,a,-a,a,a,-a,-k,k,-k,k;
                allocation*=c.mass*c.gravity/4;
                map->lower.setZero(); map->upper.setConstant(c.max_motor);
            } else {
                allocation.diagonal()<<c.mass*c.gravity,20,20,20;
                map->lower=vector(node["u_lb"],4); map->upper=vector(node["u_ub"],4);
            }
            if(!allocation.allFinite() || std::abs(allocation.determinant())<1e-30)
                throw std::invalid_argument("Singular actuator allocation");
            map->inverse=allocation.inverse(); c.control_map=map;
        } else {
            option(node,"max_acceleration",c.max_acceleration);
            option(node,"max_angular_acceleration",c.max_angular_acceleration);
        }
    }
}
Eigen::VectorXd lift(const Eigen::VectorXd &x,const YAML::Node &node,const Scene &scene) {
    Eigen::VectorXd z=Eigen::VectorXd::Zero(scene.n*scene.r);
    const auto &c=scene.model;
    if(c.robot==Robot::Unicycle) {
        rrtpff::UnicycleDF::FlatState f;
        rrtpff::UnicycleLift lift; lift.signed_velocity=node["speed"].as<double>();
        if(lift.signed_velocity*c.unicycle_branch<=0)
            throw std::invalid_argument("Unicycle lift speed and fixed branch disagree");
        if(!rrtpff::UnicycleDF::stateToFlat(x,lift,f,c.unicycle)) throw std::invalid_argument("Invalid/nonzero Unicycle lift required");
        z.head(4)=f.values;
        if(scene.r>=3) z.segment(4,2)=vector(node["acceleration"],2);
    } else if(c.robot==Robot::Quad2D) {
        if(scene.r==2) { z.head(2)=x.head(2); z.segment(2,2)=x.segment(3,2); }
        else {
            rrtpff::Quad2DDF::FlatState f;
            rrtpff::Quad2DLift lift{vector(node["acceleration"],2),vector(node["jerk"],2)};
            if(!rrtpff::Quad2DDF::stateToFlat(x,lift,f,c.quad2)) throw std::invalid_argument("Quad2D lift is inconsistent with physical boundary");
            z.head(8)=f.values; if(scene.r==5) z.tail(2)=vector(node["snap"],2);
        }
    } else if(c.robot==Robot::Quad3D) {
        if(scene.r==2) {
            rrtpff::Quad3DR2DF::FlatState f;
            if(!rrtpff::Quad3DR2DF::stateToFlat(x,{},f,c.quad3)) throw std::invalid_argument("Invalid reduced Quad3D boundary");
            z=f.values;
        } else {
            rrtpff::Quad3DDF::FlatState f; rrtpff::Quad3DLift lift;
            lift.acceleration=vector(node["acceleration"],3); lift.jerk=vector(node["jerk"],3);
            lift.yaw_rate=node["yaw_rate"].as<double>(); lift.yaw_acceleration=node["yaw_acceleration"].as<double>();
            lift.yaw_jerk=node["yaw_jerk"].as<double>();
            if(!rrtpff::Quad3DDF::stateToFlat(x,lift,f,c.quad3)) throw std::invalid_argument("Quad3D lift is inconsistent with physical boundary");
            z.head(16)=f.values;
            if(scene.r==5) { z.segment(16,3)=vector(node["snap"],3); z(19)=node["yaw_snap"].as<double>(); }
        }
    }
    return z;
}
} // namespace
void emitVector(YAML::Emitter &out,const Eigen::VectorXd &v) {
    out<<YAML::Flow<<YAML::BeginSeq; for(int k=0;k<v.size();++k) out<<v(k); out<<YAML::EndSeq;
}
Scene loadScene(const std::filesystem::path &file) {
    auto root=YAML::LoadFile(file.string()); Scene s; s.source=std::filesystem::absolute(file).string();
    const auto base=file.parent_path();
    s.robot_name=root["robot"].as<std::string>();
    if(s.robot_name=="integrator") s.model.robot=Robot::Integrator;
    else if(s.robot_name=="unicycle") s.model.robot=Robot::Unicycle;
    else if(s.robot_name=="quad2d") s.model.robot=Robot::Quad2D;
    else if(s.robot_name=="quad3d") s.model.robot=Robot::Quad3D;
    else throw std::invalid_argument("Unknown robot");
    s.n=root["n"].as<int>(s.model.robot==Robot::Quad3D ? 4:2);
    s.r=root["r"].as<int>(2);
    if(s.n<=0 || s.n>16 || s.r<=0 || s.r>7) throw std::invalid_argument("Invalid n/r");
    s.model.spatial_dim=root["spatial_dim"].as<int>(s.model.robot==Robot::Quad3D ? 3:std::min(s.n,2));
    s.model.radius=root["robot_radius"].as<double>(); // Geometry is explicit, not guessed from paper.
    option(root,"unicycle_branch",s.model.unicycle_branch);
    option(root,"constrain_sampling_bounds",s.model.constrain_sampling_bounds);
    s.model.lower=vector(root["bounds"]["lower"],s.n*s.r);
    s.model.upper=vector(root["bounds"]["upper"],s.n*s.r);
    s.weight=Eigen::MatrixXd::Identity(s.n,s.n);
    if(root["weight"]) {
        if(!root["weight"].IsSequence() || root["weight"].size()!=std::size_t(s.n)) throw std::invalid_argument("Invalid weight matrix rows");
        for(int k=0;k<s.n;++k) s.weight.row(k)=vector(root["weight"][k],s.n).transpose();
    }
    option(root,"rho",s.rho);
    if(root["dynobench"]) {
        const auto d=root["dynobench"];
        const auto env=YAML::LoadFile(resolve(base,d["environment"]).string());
        const auto model=YAML::LoadFile(resolve(base,d["model"]).string());
        s.output_step=model["dt"].as<double>(.01);
        root["dynobench_environment_snapshot"]=YAML::Clone(env);
        root["dynobench_model_snapshot"]=YAML::Clone(model);
        if(d["visual_model"]) root["dynobench_visual_model_snapshot"]=YAML::LoadFile(resolve(base,d["visual_model"]).string());
        const auto dynamics=model["dynamics"].as<std::string>();
        if((s.robot_name=="unicycle" && dynamics!="unicycle1") ||
           (s.robot_name=="quad2d" && dynamics!="quad2d") ||
           (s.robot_name=="quad3d" && dynamics!="quad3d") || s.robot_name=="integrator")
            throw std::invalid_argument("DynoBench dynamics do not match selected robot");
        robotParameters(model,s.model,true);
        if(!env["robots"].IsSequence() || env["robots"].size()!=1) throw std::invalid_argument("Exactly one DynoBench robot required");
        const int nx=s.robot_name=="unicycle" ? 3:(s.robot_name=="quad2d" ? 6:13);
        s.model.physical_start=vector(env["robots"][0]["start"],nx);
        s.model.physical_goal=vector(env["robots"][0]["goal"],nx);
        if(nx==13 && (std::abs(s.model.physical_start.segment<4>(3).norm()-1)>1e-6 ||
                      std::abs(s.model.physical_goal.segment<4>(3).norm()-1)>1e-6))
            throw std::invalid_argument("DynoBench boundaries require unit xyzw quaternions");
        s.start=lift(s.model.physical_start,root["lifting"]["start"],s);
        s.goal=lift(s.model.physical_goal,root["lifting"]["goal"],s);
        const auto lower=vector(env["environment"]["min"],s.model.spatial_dim);
        const auto upper=vector(env["environment"]["max"],s.model.spatial_dim);
        s.model.lower.head(s.model.spatial_dim)=s.model.lower.head(s.model.spatial_dim).cwiseMax(lower);
        s.model.upper.head(s.model.spatial_dim)=s.model.upper.head(s.model.spatial_dim).cwiseMin(upper);
        geometry(env["environment"],s.model,root["sphere_cell_size"].as<double>(0.2));
    } else {
        robotParameters(root["model"],s.model,false);
        s.start=vector(root["start"],s.n*s.r); s.goal=vector(root["goal"],s.n*s.r);
        if(root["environment"]) geometry(root["environment"],s.model,root["sphere_cell_size"].as<double>(0.2));
    }
    const auto p=root["planner"];
    s.planner_name=p["type"].as<std::string>("rrtconnect");
    if(s.planner_name!="rrtconnect")
        throw std::invalid_argument("Unknown planner type");
    option(p,"seed",s.planner.seed); option(p,"iterations",s.planner.iterations);
    option(p,"timeout",s.planner.timeout); option(p,"range",s.planner.range);
    option(p,"goal_bias",s.planner.goal_bias); option(p,"goal_tolerance",s.planner.goal_tolerance);
    if(p["unicycle_sample_speed"] && p["unicycle_sample_speed"].as<double>()!=0)
        throw std::invalid_argument("Pose-speed sampling is not part of this paper profile; use flat uniform sampling");
    option(p,"profile",s.planner.profile);
    option(p,"max_nodes",s.planner.max_nodes); option(p,"connect_steps",s.planner.connect_steps);
    option(p,"shortcut",s.planner.shortcut); option(p,"shortcut_attempts",s.planner.shortcut_attempts);
    if(p["metric_scale"]) s.planner.metric_scale=vector(p["metric_scale"],s.n*s.r);
    const auto v=root["validation"];
    option(v,"step",s.validation.step); option(v,"max_samples",s.validation.max_samples);
    option(v,"simd",s.validation.simd); option(v,"continuous_sphere_collision",s.validation.continuous_sphere_collision);
    option(v,"profile",s.validation.profile);
    option(v,"subdivision_depth",s.validation.subdivision_depth);
    option(v,"enforce_physical_continuity",s.validation.enforce_physical_continuity);
    option(v,"enforce_physical_endpoints",s.validation.enforce_physical_endpoints);
    option(v,"junction_tolerance",s.validation.junction_tolerance);
    option(root,"output_step",s.output_step);
    if(!std::isfinite(s.output_step) || s.output_step<=0) throw std::invalid_argument("Invalid output_step");
    if(!s.validation.enforce_physical_endpoints && !
       (s.r==2 && (s.model.robot==Robot::Quad2D || s.model.robot==Robot::Quad3D)))
        throw std::invalid_argument("Reduced endpoint contract is only supported for r2 quadrotors");
    s.configuration=YAML::Dump(root);
    return s;
}
} // namespace flask
