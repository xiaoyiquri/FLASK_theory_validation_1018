#include "fixtures.h"
using namespace theory;
int main() { return run([] {
    auto c=fixture<2,2>(); c.lower << -2,-2,-1,-1; c.upper=-c.lower;
    c.obstacles.push_back({Eigen::Vector3d(0,0,0),0.35});
    flask::Model<2,2> model(c); flask::BVP<2,2> bvp;
    flask::ValidationConfig vc; vc.step=.04; vc.continuous_sphere_collision=true;
    flask::Validator<2,2> validator(model,vc);
    flask::BVP<2,2>::State start,goal; start.values<<-1,0,0.1,0; goal.values<<1,0,0.1,0;
    const auto direct=bvp.optimal(start,goal);
    require(direct && !validator.edge(*direct),"straight BVP blocked");
    const auto mid=flask::interpolateFlat(bvp,start,goal,.25);
    require(mid && (mid->values-direct->evaluate(.25*direct->T).values).norm()<1e-10,
            "extension interpolation follows the BVP, as in the public OMPL port");
    require((mid->values-(start.values+.25*(goal.values-start.values))).norm()>.01,
            "flat-state linear interpolation is not the steering polynomial");
    const auto incoming=bvp.optimal(start,goal);
    const auto backward=flask::interpolateForTree(bvp,goal,start,.25,false);
    require(backward && (backward->values-incoming->evaluate(.75*incoming->T).values).norm()<1e-10,
            "goal extension follows incoming BVP suffix in positive time");
    for(double fraction:{.1,.3,.8}) {
        const auto prefix=flask::interpolateForTree(bvp,start,goal,fraction,true);
        const auto suffix=flask::interpolateForTree(bvp,goal,start,fraction,false);
        require(prefix && suffix,"both range truncations exist");
        const auto outgoing=bvp.optimal(start,*prefix),ingoing=bvp.optimal(*suffix,goal);
        require(outgoing && ingoing && outgoing->T>0 && ingoing->T>0,"re-steered edges have positive time");
        require((outgoing->evaluate(0).values-start.values).norm()<1e-8 &&
                (outgoing->evaluate(outgoing->T).values-prefix->values).norm()<1e-8,"outgoing re-steer meets both complete endpoints");
        require((ingoing->evaluate(0).values-suffix->values).norm()<1e-8 &&
                (ingoing->evaluate(ingoing->T).values-goal.values).norm()<1e-8,"incoming re-steer points from new node to goal parent");
    }
    const auto scales=flask::metricScale(model);
    const double expectedMetric=(start.values.head<2>()-goal.values.head<2>()).norm()/
        (c.upper.head<2>()-c.lower.head<2>()).norm();
    relative(flask::flatMetric<2,2>(start,goal,scales),expectedMetric,1e-12,
             "default extent-weighted flat metric");
    flask::PlannerConfig config; config.seed=41; config.iterations=3000; config.timeout=8;
    config.range=1; config.shortcut=true;
    flask::RRTConnect<2,2> planner(model,bvp,validator,config);
    const auto result=planner.solve(start,goal);
    require(result.success,"RRTConnect must find an obstacle detour");
    require(result.nodes>2 && result.curves.size()>=2,"actual bidirectional tree expansion");
    require(validator.path(result.curves,start,goal),"both trees' edges form a forward-time path");
    require(result.cost()<=result.cost_before_shortcut+1e-8,"shortcut never increases cost");
    require(result.history.size()==result.shortcuts+1 && !result.first_curves.empty(),"first solution and every accepted shortcut captured");
    double previousCost=INFINITY,previousTime=-1;
    for(const auto &event:result.history) {
        flask::Plan<2,2> snapshot; snapshot.curves=event.curves;
        require(event.seconds>=previousTime && event.seconds<=config.timeout,"monotonic in-budget observation times");
        require(snapshot.cost()<previousCost,"incumbent objective strictly decreases");
        require(validator.path(event.curves,start,goal),"every recorded path has correct forward-time traceback and endpoints");
        previousCost=snapshot.cost(); previousTime=event.seconds;
    }
    relative(result.history.front().seconds,result.search_seconds,1e-12,"first time belongs to original RRTConnect path");
    flask::Plan<2,2> raw;raw.curves=result.first_curves;
    relative(raw.cost(),result.cost_before_shortcut,1e-12,"raw first cost remains unchanged by shortcut");
    relative(previousCost,result.cost(),1e-12,"last history snapshot is the reported final path");
    // Same seed and enough iteration budget must reproduce the selected path.
    flask::Validator<2,2> secondValidator(model,vc);
    flask::RRTConnect<2,2> second(model,bvp,secondValidator,config);
    const auto repeated=second.solve(start,goal);
    require(repeated.success && repeated.nodes==result.nodes,"deterministic tree under fixed seed");
    relative(repeated.cost(),result.cost(),1e-12,"deterministic path cost");
    auto bad=start; bad.values.head<2>().setZero();
    require(!planner.solve(bad,goal).success,"colliding endpoint rejected");
    auto wall=c; wall.obstacles={{Eigen::Vector3d::Zero(),3}};
    flask::Model<2,2> blocked(wall); flask::Validator<2,2> reject(blocked);
    flask::RRTConnect<2,2> cannot(blocked,bvp,reject,config);
    require(!cannot.solve(start,goal).success,"no fabricated fallback path");
    auto tiny=config; tiny.timeout=1e-9;
    flask::RRTConnect<2,2> deadlineLimited(model,bvp,validator,tiny);
    const auto late=deadlineLimited.solve(start,goal);
    require(!late.success && late.history.empty(),"no first solution can be published after the deadline");
    std::cout<<"detour nodes="<<result.nodes<<" cost="<<result.cost()<<" shortcuts="<<result.shortcuts<<'\n';
}); }
