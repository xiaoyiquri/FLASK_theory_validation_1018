#pragma once
#include "flask/validation.h"
#include <chrono>
#include <random>
#include <numeric>

namespace flask {
template<int N,int R> Eigen::VectorXd metricScale(const Model<N,R> &model) {
    Eigen::VectorXd scale(N*R);
    for(int d=0;d<R;++d) {
        const double extent=(model.config().upper.segment(d*N,N)-model.config().lower.segment(d*N,N)).norm();
        scale.segment(d*N,N).setConstant(extent>1e-12 ? extent:1);
    }
    return scale;
}
template<int N,int R> double flatMetric(const typename BVP<N,R>::State &a,
    const typename BVP<N,R>::State &b,const Eigen::VectorXd &scale) {
    double sum=0;
    for(int d=0;d<R;++d)
        sum+=((a.values.template segment<N>(d*N)-b.values.template segment<N>(d*N)).array()/
              scale.segment(d*N,N).array()).matrix().norm();
    return sum;
}
template<int N,int R> std::optional<typename BVP<N,R>::State> interpolateFlat(
    const BVP<N,R> &bvp,const typename BVP<N,R>::State &from,
    const typename BVP<N,R>::State &to,double fraction) {
    if(!std::isfinite(fraction) || fraction<0 || fraction>1)
        throw std::invalid_argument("Interpolation fraction outside [0,1]");
    if(fraction==0 || (from.values-to.values).norm()<1e-12) return from;
    if(fraction==1) return to;
    const auto motion=bvp.optimal(from,to);
    if(!motion) return {};
    return motion->evaluate(fraction*motion->T);
}
template<int N,int R> std::optional<typename BVP<N,R>::State> interpolateForTree(
    const BVP<N,R> &bvp,const typename BVP<N,R>::State &parent,
    const typename BVP<N,R>::State &target,double fraction,bool forward) {
    // A goal-tree edge must be a suffix of target -> parent, not a
    // prefix of parent -> target. Do not integrate with negative time.
    return forward ? interpolateFlat(bvp,parent,target,fraction):
                     interpolateFlat(bvp,target,parent,1-fraction);
}
struct PlannerConfig {
    std::uint32_t seed=41;
    std::size_t iterations=10000,connect_steps=0,max_nodes=20000;
    double timeout=5,range=1,goal_bias=0,goal_tolerance=0.1;
    Eigen::VectorXd metric_scale;
    bool shortcut=true;
    bool profile=false;
    std::size_t shortcut_attempts=100;
};
template<int N,int R> struct Incumbent {
    double seconds=0;
    std::string stage;
    std::vector<typename BVP<N,R>::Curve> curves;
};
template<int N,int R> struct Plan {
    bool success=false;
    std::string reason;
    std::vector<typename BVP<N,R>::Curve> curves;
    std::vector<typename BVP<N,R>::Curve> first_curves;
    std::vector<Incumbent<N,R>> history;
    std::size_t iterations=0,nodes=0,bvp_queries=0,shortcuts=0;
    double seconds=0,search_seconds=0,cost_before_shortcut=0;
    double nearest_seconds=0,bvp_seconds=0;
    std::size_t extension_attempts=0,valid_extensions=0;
    double cost() const { double sum=0; for(const auto &c:curves) sum+=c.cost; return sum; }
    double duration() const { double sum=0; for(const auto &c:curves) sum+=c.T; return sum; }
    // Composite Simpson integration of translational speed, excluding yaw.
    double length(int spatial_dim,bool first=false) const {
        double sum=0;
        for(const auto &c:(first ? first_curves:curves)) {
            const int intervals=std::max(100,2*int(std::ceil(c.T/.002)));
            double integral=0;
            for(int i=0;i<=intervals;++i) {
                const double speed=c.derivative(c.T*(double(i)/intervals),1).head(spatial_dim).norm();
                integral+=(i==0 || i==intervals ? 1:(i%2 ? 4:2))*speed;
            }
            sum+=integral*c.T/(3*intervals);
        }
        return sum;
    }
};
template<int N,int R> class RRTConnect {
public:
    using State=typename BVP<N,R>::State;
    using Curve=typename BVP<N,R>::Curve;
    using Clock=std::chrono::steady_clock;
    RRTConnect(const Model<N,R> &model,const BVP<N,R> &bvp,Validator<N,R> &validator,
               PlannerConfig config={}):model_(model),bvp_(bvp),validator_(validator),config_(std::move(config)) {
        if(config_.metric_scale.size()==0) config_.metric_scale=metricScale(model_);
        if(config_.metric_scale.size()!=N*R || !config_.metric_scale.allFinite() ||
           (config_.metric_scale.array()<=0).any() || !std::isfinite(config_.range) || config_.range<=0 ||
           !std::isfinite(config_.timeout) || config_.timeout<=0 || config_.iterations==0 ||
           config_.max_nodes<2 || !std::isfinite(config_.goal_bias) ||
           config_.goal_bias<0 || config_.goal_bias>1)
            throw std::invalid_argument("Invalid RRTConnect configuration");
    }
    Plan<N,R> solve(const State &start,const State &goal) {
        const auto begin=Clock::now();
        begin_=begin;
        deadline_=begin+std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(config_.timeout));
        Plan<N,R> result;
        auto finish=[&](std::string reason) {
            result.reason=std::move(reason); result.seconds=std::chrono::duration<double>(Clock::now()-begin).count();
            if(result.search_seconds==0) result.search_seconds=result.seconds;
            return std::move(result);
        };
        if(!model_.nodeValid(start)||!model_.nodeValid(goal)) return finish("invalid_endpoint");
        if((start.values-goal.values).norm()<1e-12) {
            result.success=validator_.path({},start,goal);
            if(result.success && !expired()) record(result,"first_solution");
            else if(result.success) { result.success=false; return finish("timeout"); }
            return finish(result.success ? "already_at_goal":"physical_boundary_mismatch");
        }
        ++result.bvp_queries;
        auto direct=steer(start,goal,result);
        if(direct && validator_.path({*direct},start,goal)) {
            const auto observed=Clock::now();
            if(observed>=deadline_) return finish("timeout");
            result.success=true; result.curves={*direct}; result.first_curves=result.curves; result.nodes=2;
            result.cost_before_shortcut=result.cost(); record(result,"first_solution",observed); return finish("direct_bvp");
        }
        std::vector<Node> from{{start,0,{}}},to{{goal,0,{}}};
        std::mt19937 rng(config_.seed); std::uniform_real_distribution<double> unit(0,1);
        for(std::size_t iteration=0;iteration<config_.iterations && !expired();++iteration) {
            result.iterations=iteration+1;
            if(from.size()+to.size()>=config_.max_nodes) { result.nodes=from.size()+to.size(); return finish("node_limit"); }
            const bool forward=iteration%2==0;
            auto &active=forward ? from:to; auto &other=forward ? to:from;
            State target;
            if(unit(rng)<config_.goal_bias) target=forward ? goal:start;
            else {
                for(int k=0;k<N*R;++k) target.values(k)=model_.config().lower(k)+
                    unit(rng)*(model_.config().upper(k)-model_.config().lower(k));
            }
            // A sampled target may be invalid while the range-limited edge
            // toward it is valid. RRTConnect checks the actual new state and
            // motion, not the raw uniform sample (public OMPL growTree).
            const auto grown=extend(active,target,forward,result);
            if(grown.status==Trapped) continue;
            const State meet=active[grown.index].state;
            Extension joined;
            for(std::size_t k=0;(!config_.connect_steps || k<config_.connect_steps) && !expired();++k) {
                if(from.size()+to.size()>=config_.max_nodes) break;
                joined=extend(other,meet,!forward,result);
                if(joined.status!=Advanced) break;
            }
            if(joined.status!=Reached) continue;
            const auto si=forward ? grown.index:joined.index;
            const auto gi=forward ? joined.index:grown.index;
            auto path=extract(from,to,si,gi);
            if(!validator_.path(path,start,goal)) continue;
            const auto observed=Clock::now();
            if(observed>=deadline_) break;
            result.success=true; result.curves=std::move(path); result.nodes=from.size()+to.size();
            result.cost_before_shortcut=result.cost();
            result.first_curves=result.curves;
            record(result,"first_solution",observed);
            if(config_.shortcut) shorten(result,start,goal);
            return finish("connected");
        }
        result.nodes=from.size()+to.size();
        return finish(expired() ? "timeout":"iteration_limit");
    }
    void shorten(Plan<N,R> &plan,const State &start,const State &goal) {
        if(!plan.success || plan.curves.size()<2) return;
        if(deadline_==Clock::time_point::max()) deadline_=Clock::now()+
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(config_.timeout));
        std::size_t attempts=0;
        for(std::size_t i=0;i<plan.curves.size() && !expired();++i) {
            std::size_t j=plan.curves.size();
            while(j>i+1 && !expired()) {
                if(++attempts>config_.shortcut_attempts) return;
                ++plan.bvp_queries;
                const auto a=plan.curves[i].evaluate(0),b=plan.curves[j-1].evaluate(plan.curves[j-1].T);
                auto candidate=steer(a,b,plan); if(!candidate) { --j; continue; }
                double old=0; for(std::size_t k=i;k<j;++k) old+=plan.curves[k].cost;
                if(candidate->cost>=old-1e-9) { --j; continue; }
                auto proposed=plan.curves;
                proposed.erase(proposed.begin()+i,proposed.begin()+j);
                proposed.insert(proposed.begin()+i,*candidate);
                if(!validator_.path(proposed,start,goal)) { --j; continue; }
                const auto observed=Clock::now();
                if(observed>=deadline_) return; // Do not publish an incumbent found after its budget.
                plan.curves=std::move(proposed); ++plan.shortcuts;
                record(plan,"shortcut",observed);
                // Splicing changes M. Revisit this start node with current
                // indices, rather than skipping the newly adjacent candidates.
                j=plan.curves.size();
            }
        }
    }
private:
    void record(Plan<N,R> &plan,const char *stage,Clock::time_point observed=Clock::now()) const {
        const double seconds=std::chrono::duration<double>(observed-begin_).count();
        if(plan.history.empty()) plan.search_seconds=seconds;
        plan.history.push_back({seconds,stage,plan.curves});
    }
    std::optional<Curve> steer(const State &from,const State &to,Plan<N,R> &result) const {
        ProfileScope timer(config_.profile,result.bvp_seconds); return bvp_.optimal(from,to);
    }
    struct Node { State state; std::size_t parent; std::optional<Curve> edge; };
    enum Status { Trapped,Advanced,Reached };
    struct Extension { Status status=Trapped; std::size_t index=0; };
    double distance(const State &a,const State &b) const {
        return flatMetric<N,R>(a,b,config_.metric_scale);
    }
    bool expired() const { return Clock::now()>=deadline_; }
    Extension extend(std::vector<Node> &tree,const State &target,bool forward,Plan<N,R> &result) {
        if(expired()) return {};
        ++result.extension_attempts;
        std::size_t best=0; double nearest=INFINITY;
        { ProfileScope timer(config_.profile,result.nearest_seconds);
        for(std::size_t i=0;i<tree.size();++i) {
            const double d=distance(tree[i].state,target); if(d<nearest) { best=i; nearest=d; }
        }
        }
        // Reached requires matching flat states, not just a small scaled metric.
        if((tree[best].state.values-target.values).norm()<1e-10) return {Reached,best};
        State endpoint=target;
        if(nearest>config_.range) {
            // Kinodynamic adaptation: outgoing prefix at start, incoming
            // suffix at goal. The public geometric port interpolates
            // nearest -> sample in both trees; that is not used here.
            ++result.bvp_queries;
            std::optional<State> limited;
            { ProfileScope timer(config_.profile,result.bvp_seconds);
              limited=interpolateForTree(bvp_,tree[best].state,target,config_.range/nearest,forward); }
            if(!limited) return {};
            endpoint=*limited;
            if(distance(tree[best].state,endpoint)<1e-8) return {};
        }
        if(!model_.nodeValid(endpoint)) return {};
        ++result.bvp_queries;
        // Goal-tree curves run new -> parent -> goal, all in positive time.
        auto curve=forward ? steer(tree[best].state,endpoint,result):steer(endpoint,tree[best].state,result);
        if(!curve || !validator_.edge(*curve)) return {};
        tree.push_back({endpoint,best,std::move(curve)});
        ++result.valid_extensions;
        return {nearest<=config_.range ? Reached:Advanced,tree.size()-1};
    }
    static std::vector<Curve> extract(const std::vector<Node> &from,const std::vector<Node> &to,
                                     std::size_t si,std::size_t gi) {
        std::vector<Curve> path;
        while(si) { path.push_back(*from[si].edge); si=from[si].parent; }
        std::reverse(path.begin(),path.end());
        while(gi) { path.push_back(*to[gi].edge); gi=to[gi].parent; }
        return path;
    }
    const Model<N,R> &model_;
    const BVP<N,R> &bvp_;
    Validator<N,R> &validator_;
    PlannerConfig config_;
    Clock::time_point deadline_=Clock::time_point::max();
    Clock::time_point begin_=Clock::now();
};
} // namespace flask
