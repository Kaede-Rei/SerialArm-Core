#pragma once
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <utility>
#include <vector>

namespace serial_arm::servo_tuning {
// A constant stationary trace does not identify an encoder quantum. Maintain
// the same minimum adjacent positive difference as observed_encoder_step,
// using only ordered fresh feedback. In particular, neither the largest
// movement increment nor a repeated feedback packet becomes a noise estimate.
class ObservedPositionStep {
public:
    bool observe(double position,std::uint64_t sequence,bool fresh=true) {
        if(!fresh || !sequence || sequence<=last_sequence_ || !std::isfinite(position))return false;
        last_sequence_=sequence;++fresh_samples_;
        const auto following=positions_.lower_bound(position);
        if(following!=positions_.end() && *following==position)return true;
        const auto previous=following==positions_.begin()?positions_.end():std::prev(following);
        if(previous!=positions_.end() && following!=positions_.end())remove_gap(*following-*previous);
        if(previous!=positions_.end())add_gap(position-*previous);
        if(following!=positions_.end())add_gap(*following-position);
        positions_.insert(following,position);return true;
    }
    double step() const {return gaps_.empty()?0:*gaps_.begin();}
    std::size_t distinct_positions() const {return positions_.size();}
    // One displacement is only a sampled motion increment. Repeated adjacent
    // spacings across at least three distinct codes support a resolution
    // estimate; a consumer must still bound an initially unknown estimate by
    // its independent motion scale so sparse large moves cannot mask motion.
    std::size_t step_evidence_pairs() const {
        const double value=step();if(value<=0)return 0;
        const double tolerance=std::max(1e-9,.01*value);
        // Two pairs are sufficient. Cap this diagnostic at two so a long
        // trajectory with thousands of equal gaps stays constant-cost here.
        std::size_t count=0;
        for(auto it=gaps_.begin();it!=gaps_.end() && *it<=value+tolerance && count<2;++it)++count;
        return count;
    }
    double qualified_step() const {
        const double value=step();
        if(fresh_samples_<8 || distinct_positions()<3 || step_evidence_pairs()<2)return 0;
        // Turning points of a smooth analog wave also produce two equal small
        // increments. Require the other observed gap sizes to share the same
        // lattice instead of mistaking these local increments for finer codes.
        // Bounded distinct-gap checks avoid a full trace scan on the control
        // thread; a richer unresolved trace stays unknown, never becomes noise.
        auto it=gaps_.begin();
        for(unsigned groups=0;it!=gaps_.end() && groups<8;++groups) {
            const double gap=*it,ratio=gap/value;
            if(std::abs(ratio-std::round(ratio))>.05)return 0;
            it=gaps_.upper_bound(gap+.01*value);
        }
        return it==gaps_.end()?value:0;
    }
    std::size_t fresh_samples() const {return fresh_samples_;}
    std::uint64_t last_sequence() const {return last_sequence_;}
private:
    void add_gap(double gap){if(gap>1e-9)gaps_.insert(gap);}
    void remove_gap(double gap){if(gap>1e-9){const auto found=gaps_.find(gap);if(found!=gaps_.end())gaps_.erase(found);}}
    std::set<double> positions_;
    std::multiset<double> gaps_;
    std::uint64_t last_sequence_{0};
    std::size_t fresh_samples_{0};
};
struct MeasurementPathFeedbackReserve {
    bool valid{false},encoder_step_known{false};
    double position_uncertainty{0};
    std::string reason;
};
inline MeasurementPathFeedbackReserve measurement_path_feedback_reserve(double lead,double observed_step,double stationary_rms) {
    MeasurementPathFeedbackReserve out;
    if(!std::isfinite(lead) || lead<=0 || !std::isfinite(observed_step) || observed_step<0 ||
       !std::isfinite(stationary_rms) || stationary_rms<0){out.reason="measurement_path_feedback_reserve_input_invalid";return out;}
    out.encoder_step_known=observed_step>0;
    out.position_uncertainty=out.encoder_step_known?std::max({3*observed_step,6*stationary_rms,1e-6}):std::max(.25*lead,6*stationary_rms);
    out.valid=out.position_uncertainty<lead;
    out.reason=!out.valid?"measurement_path_position_resolution_or_noise_consumes_feedback_lead":
        out.encoder_step_known?"observed_position_step_and_stationary_noise_reserve":"unidentified_position_step_internal_guard_buffer";
    return out;
}

struct SimplifiedDemonstratedRoute {
    bool valid{false};std::size_t original_nodes{0};
    std::vector<std::vector<double>> route;
    std::vector<std::size_t> source_indices;
    std::vector<double> tolerance_rad;
};
// Retain complete original poses and their source indices. Encoder sub-code
// deviations may be redundant geometry; an unknown encoder permits only
// numerical-roundoff collinearity, never an invented physical tolerance.
inline SimplifiedDemonstratedRoute simplify_demonstrated_route(const std::vector<std::vector<double>>& route,
    const std::vector<std::size_t>& source_indices,const std::vector<double>& observed_position_step_rad={},
    const std::vector<std::size_t>& preserved_nodes={}) {
    SimplifiedDemonstratedRoute out;out.original_nodes=route.size();
    if(route.empty() || route.front().empty() || source_indices.size()!=route.size())return out;
    const auto n=route.front().size();
    if(!observed_position_step_rad.empty() && observed_position_step_rad.size()!=n)return out;
    out.tolerance_rad.assign(n,0);
    for(std::size_t i=0;i<n;++i) {
        const double step=observed_position_step_rad.empty()?0:observed_position_step_rad[i];
        if(!std::isfinite(step) || step<0)return out;
        double magnitude=1;
        for(const auto& pose:route) {
            if(pose.size()!=n || !std::isfinite(pose[i]))return out;
            magnitude=std::max(magnitude,std::abs(pose[i]));
        }
        out.tolerance_rad[i]=std::max(.5*step,64*std::numeric_limits<double>::epsilon()*magnitude);
    }
    std::vector<bool> keep(route.size(),false);keep.front()=keep.back()=true;
    for(const auto index:preserved_nodes){if(index>=route.size())return out;keep[index]=true;}
    std::vector<std::pair<std::size_t,std::size_t>> intervals;
    std::size_t previous=0;
    for(std::size_t k=1;k<route.size();++k)if(keep[k]){intervals.emplace_back(previous,k);previous=k;}
    while(!intervals.empty()) {
        const auto [a,b]=intervals.back();intervals.pop_back();if(b-a<2)continue;
        long double denominator=0,l1=0;
        for(std::size_t i=0;i<n;++i) {
            const long double d=(static_cast<long double>(route[b][i])-route[a][i])/out.tolerance_rad[i];
            denominator+=d*d;l1+=std::abs(d);
        }
        double worst=1;std::size_t split=0,furthest_index=a;
        long double furthest_progress=0;
        // Spatial RDP alone erases a real collinear out-and-back motion.
        // Preserve a backwards excursion larger than the same per-axis
        // resolution allowance, even when every pose lies on the chord.
        const long double progress_tolerance=denominator>0?l1/denominator:0;
        for(std::size_t k=a+1;k<b;++k) {
            long double numerator=0;
            for(std::size_t i=0;i<n;++i)numerator+=
                ((static_cast<long double>(route[k][i])-route[a][i])/out.tolerance_rad[i])*
                ((static_cast<long double>(route[b][i])-route[a][i])/out.tolerance_rad[i]);
            const long double progress=denominator>0?numerator/denominator:0;
            const long double clipped=std::clamp(progress,0.0L,1.0L);
            double deviation=0;
            for(std::size_t i=0;i<n;++i) {
                const long double expected=route[a][i]+clipped*(static_cast<long double>(route[b][i])-route[a][i]);
                deviation=std::max(deviation,static_cast<double>(std::abs(route[k][i]-expected)/out.tolerance_rad[i]));
            }
            if(deviation>worst){worst=deviation;split=k;}
            if(progress>furthest_progress){furthest_progress=progress;furthest_index=k;}
            else if(!split && furthest_index>a && furthest_progress-progress>progress_tolerance)split=furthest_index;
        }
        if(split){keep[split]=true;intervals.emplace_back(a,split);intervals.emplace_back(split,b);}
    }
    for(std::size_t k=0;k<route.size();++k)if(keep[k]){out.route.push_back(route[k]);out.source_indices.push_back(source_indices[k]);}
    out.valid=true;return out;
}
// Geometry only. Safety bounds are explicit user limits, not dynamics or proof
// of reachable hard stops. Every selected pose belongs to the demonstrated
// ordered polyline; its entry bridge uses the existing alignment allowance.
struct DemonstratedCenterRequest {
    std::vector<double> fresh_position,safety_lower,safety_upper;
    std::vector<double> desired_room,entry_tolerance;
    std::vector<bool> priority_axes;
    bool reverse{false};
    std::vector<double> observed_position_step_rad;
};
struct DemonstratedCenterPlan {
    bool valid{false},migration_required{false},reverse{false};
    std::vector<std::vector<double>> route;
    std::vector<std::size_t> source_indices;
    std::vector<std::vector<double>> demonstrated_route;
    std::vector<std::size_t> demonstrated_source_indices;
    std::vector<double> demonstrated_lower,demonstrated_upper;
    std::vector<double> center,available_room;
    std::vector<bool> room_satisfied;
    std::vector<std::size_t> insufficient_axes;
    std::vector<double> observed_position_step_rad,route_simplification_tolerance_rad;
    std::size_t route_original_nodes{0},demonstrated_route_original_nodes{0};
    std::size_t entry_source_index{0},target_source_index{0};
    double entry_fraction{0};
    std::string reason;
    std::string entry_scope{"fresh_feedback_bridge_with_existing_alignment_tolerance_then_ordered_demonstrated_path"};
};
namespace measurement_path_detail {
inline bool interval(double origin,double delta,double lower,double upper,double& first,double& last) {
    if(delta==0)return origin>=lower && origin<=upper;
    double a=(lower-origin)/delta,b=(upper-origin)/delta;if(a>b)std::swap(a,b);
    first=std::max(first,a);last=std::min(last,b);return first<=last;
}
inline std::vector<double> point(const std::vector<double>& a,const std::vector<double>& b,double u) {
    if(u<=0)return a;
    if(u>=1)return b;
    std::vector<double> q(a.size());for(std::size_t i=0;i<q.size();++i)q[i]=a[i]+u*(b[i]-a[i]);return q;
}
inline bool finite(const std::vector<double>& v) {
    return std::all_of(v.begin(),v.end(),[](double x){return std::isfinite(x);});
}
}
inline DemonstratedCenterPlan plan_demonstrated_measurement_center(
    const std::vector<std::vector<double>>& path,const DemonstratedCenterRequest& req) {
    DemonstratedCenterPlan out;out.reverse=req.reverse;
    const auto n=req.fresh_position.size(),count=path.size();
    const auto reject=[&](const char* why){out.reason=why;return out;};
    if(!n || count<2 || req.safety_lower.size()!=n || req.safety_upper.size()!=n ||
       req.desired_room.size()!=n || req.entry_tolerance.size()!=n ||
       (!req.priority_axes.empty() && req.priority_axes.size()!=n) ||
       (!req.observed_position_step_rad.empty() && req.observed_position_step_rad.size()!=n))return reject("measurement_center_mapping_invalid");
    if(!measurement_path_detail::finite(req.fresh_position))return reject("measurement_center_fresh_position_invalid");
    std::vector<double> lower(n,std::numeric_limits<double>::infinity()),upper(n,-std::numeric_limits<double>::infinity());
    for(const auto& q:path) {
        if(q.size()!=n || !measurement_path_detail::finite(q))return reject("measurement_center_demonstrated_path_invalid");
        for(std::size_t i=0;i<n;++i){lower[i]=std::min(lower[i],q[i]);upper[i]=std::max(upper[i],q[i]);}
    }
    const auto source=[&](std::size_t k){return req.reverse?count-1-k:k;};
    const auto& endpoint=path[source(0)];
    std::vector<int> inward(n,0);std::vector<bool> wanted(n,false),feasible(n,false);
    for(std::size_t i=0;i<n;++i) {
        if(std::isnan(req.safety_lower[i]) || std::isnan(req.safety_upper[i]) || req.safety_lower[i]>req.safety_upper[i] ||
           !std::isfinite(req.desired_room[i]) || req.desired_room[i]<0 || !std::isfinite(req.entry_tolerance[i]) || req.entry_tolerance[i]<=0)
            return reject("measurement_center_bounds_invalid");
        if(req.fresh_position[i]<req.safety_lower[i] || req.fresh_position[i]>req.safety_upper[i])return reject("measurement_center_fresh_position_outside_safety");
        if(std::abs(endpoint[i]-req.fresh_position[i])>req.entry_tolerance[i])return reject("measurement_center_outside_existing_endpoint_alignment");
        lower[i]=std::max(lower[i],req.safety_lower[i]);upper[i]=std::min(upper[i],req.safety_upper[i]);
        if(lower[i]>upper[i])return reject("measurement_center_no_demonstrated_safety_intersection");
        feasible[i]=req.desired_room[i]>0 && upper[i]-lower[i]>=2*req.desired_room[i];
        wanted[i]=req.desired_room[i]>0 && (req.priority_axes.empty() || req.priority_axes[i]);
        if(wanted[i]) {
            const double left=req.fresh_position[i]-lower[i],right=upper[i]-req.fresh_position[i];
            if(left<req.desired_room[i] && left<right)inward[i]=1;
            else if(right<req.desired_room[i] && right<left)inward[i]=-1;
        }
    }
    // Project only onto the contiguous, near-end portion accepted by the
    // existing guide. A later return to this area is not an entry shortcut.
    double best_distance=std::numeric_limits<double>::infinity(),entry_u=0;
    std::size_t entry_segment=0;bool entry_found=false;
    const auto find_entry=[&](std::size_t minimum_segment,std::size_t maximum_segment) {
      entry_found=false;best_distance=std::numeric_limits<double>::infinity();
      for(std::size_t k=0;k+1<count && k<=maximum_segment;++k) {
        const auto& a=path[source(k)];const auto& b=path[source(k+1)];
        double first=0,last=1;
        for(std::size_t i=0;i<n && first<=last;++i)
            if(!measurement_path_detail::interval(a[i],b[i]-a[i],
                std::max(req.safety_lower[i],req.fresh_position[i]-req.entry_tolerance[i]),
                std::min(req.safety_upper[i],req.fresh_position[i]+req.entry_tolerance[i]),first,last)){first=1;last=0;}
        if(first>last)break;
        double allowed_first=first,allowed_last=last;
        for(std::size_t i=0;i<n && allowed_first<=allowed_last;++i)if(inward[i]) {
            const double lo=inward[i]>0?req.fresh_position[i]:-std::numeric_limits<double>::infinity();
            const double hi=inward[i]<0?req.fresh_position[i]:std::numeric_limits<double>::infinity();
            if(!measurement_path_detail::interval(a[i],b[i]-a[i],lo,hi,allowed_first,allowed_last)){allowed_first=1;allowed_last=0;}
        }
        if(k>=minimum_segment && allowed_first<=allowed_last) {
            double numerator=0,denominator=0;
            for(std::size_t i=0;i<n;++i){const double d=(b[i]-a[i])/req.entry_tolerance[i];numerator+=d*(req.fresh_position[i]-a[i])/req.entry_tolerance[i];denominator+=d*d;}
            const double u=std::clamp(denominator>0?numerator/denominator:allowed_first,allowed_first,allowed_last);
            const auto q=measurement_path_detail::point(a,b,u);double distance=0;
            for(std::size_t i=0;i<n;++i){const double e=(q[i]-req.fresh_position[i])/req.entry_tolerance[i];distance+=e*e;}
            if(distance<best_distance){best_distance=distance;entry_segment=k;entry_u=u;entry_found=true;}
        }
        if(last<1)break;
      }
    };
    find_entry(0,count-2);
    if(!entry_found)return reject("measurement_center_no_inward_entry_within_existing_alignment");
    auto projected=measurement_path_detail::point(path[source(entry_segment)],path[source(entry_segment+1)],entry_u);
    std::vector<bool> required(n,false);bool any_required=false;
    for(std::size_t i=0;i<n;++i){required[i]=wanted[i] && feasible[i];any_required=any_required || required[i];}
    if(!any_required)for(std::size_t i=0;i<n;++i){required[i]=feasible[i];any_required=any_required || required[i];}
    const auto rooms=[&](const std::vector<double>& q){std::vector<double> r(n);for(std::size_t i=0;i<n;++i)r[i]=std::max(0.0,std::min(q[i]-lower[i],upper[i]-q[i]));return r;};
    const auto score=[&](const std::vector<double>& q){const auto room=rooms(q);double minimum=1;std::size_t complete=0;
        for(std::size_t i=0;i<n;++i)if(required[i]){const double ratio=room[i]/req.desired_room[i];minimum=std::min(minimum,ratio);complete+=ratio>=1;}
        return std::pair<std::size_t,double>{complete,minimum};};
    std::size_t required_count=0;for(bool value:required)required_count+=value;
    auto best_score=score(projected);std::size_t target=entry_segment;bool target_is_projection=true;out.center=projected;
    if(any_required && best_score.first!=required_count)for(std::size_t k=entry_segment+1;k<count;++k) {
        const auto& q=path[source(k)];bool safe=true;
        for(std::size_t i=0;i<n;++i)safe=safe && q[i]>=req.safety_lower[i] && q[i]<=req.safety_upper[i];
        if(!safe)break;
        const auto candidate=score(q);
        if(candidate>best_score){best_score=candidate;out.center=q;target=k;target_is_projection=false;}
        if(candidate.first==required_count)break;
    }
    // A recorded endpoint can contain a small inward/outward wiggle. Join
    // after its last outward sample before this target, while the entire entry
    // bridge remains inside the same original alignment allowance. Do not let
    // a later return in the demonstration change an already adequate target.
    if(!target_is_projection) {
        std::size_t minimum_segment=entry_segment;
        for(std::size_t k=entry_segment+1;k<=target;++k)for(std::size_t i=0;i<n;++i)
            if(inward[i] && inward[i]*(path[source(k)][i]-req.fresh_position[i])<0)minimum_segment=k;
        if(minimum_segment>entry_segment) {
            find_entry(minimum_segment,target-1);
            if(!entry_found)return reject("measurement_center_no_inward_route_within_existing_alignment");
            projected=measurement_path_detail::point(path[source(entry_segment)],path[source(entry_segment+1)],entry_u);
        }
    }
    out.entry_source_index=source(entry_segment);out.entry_fraction=entry_u;
    out.available_room=rooms(out.center);out.room_satisfied.resize(n);
    for(std::size_t i=0;i<n;++i){out.room_satisfied[i]=out.available_room[i]>=req.desired_room[i];if(!out.room_satisfied[i])out.insufficient_axes.push_back(i);}
    out.target_source_index=target_is_projection?source(entry_segment):source(target);
    out.demonstrated_lower=lower;out.demonstrated_upper=upper;
    const auto append_demonstrated=[&](const std::vector<double>& q,std::size_t index){
        if(out.demonstrated_route.empty() || out.demonstrated_route.back()!=q){out.demonstrated_route.push_back(q);out.demonstrated_source_indices.push_back(index);}};
    const auto last_original=target_is_projection?entry_segment:target;
    for(std::size_t k=0;k<=last_original;++k) {
        const auto& q=path[source(k)];
        for(std::size_t i=0;i<n;++i)if(q[i]<req.safety_lower[i] || q[i]>req.safety_upper[i])
            return reject("measurement_center_original_endpoint_route_outside_safety");
        append_demonstrated(q,source(k));
    }
    if(target_is_projection)append_demonstrated(projected,source(entry_segment));
    const auto append=[&](const std::vector<double>& q,std::size_t index){if(out.route.empty() || out.route.back()!=q){out.route.push_back(q);out.source_indices.push_back(index);}};
    append(req.fresh_position,std::numeric_limits<std::size_t>::max());
    append(projected,source(entry_segment));
    if(!target_is_projection)for(std::size_t k=entry_segment+1;k<=target;++k)append(path[source(k)],source(k));
    const std::vector<std::size_t> preserve_bridge=req.fresh_position!=projected?std::vector<std::size_t>{1}:std::vector<std::size_t>{};
    const auto simplified=simplify_demonstrated_route(out.route,out.source_indices,req.observed_position_step_rad,preserve_bridge);
    const auto demonstrated=simplify_demonstrated_route(out.demonstrated_route,out.demonstrated_source_indices,req.observed_position_step_rad);
    if(!simplified.valid || !demonstrated.valid)return reject("measurement_center_position_resolution_invalid");
    out.route_original_nodes=simplified.original_nodes;out.demonstrated_route_original_nodes=demonstrated.original_nodes;
    out.route=simplified.route;out.source_indices=simplified.source_indices;
    out.demonstrated_route=demonstrated.route;out.demonstrated_source_indices=demonstrated.source_indices;
    out.observed_position_step_rad=req.observed_position_step_rad;
    if(out.observed_position_step_rad.empty())out.observed_position_step_rad.assign(n,0);
    out.route_simplification_tolerance_rad=simplified.tolerance_rad;
    out.migration_required=out.route.size()>1;out.valid=true;
    out.reason=!any_required?"demonstrated_path_has_no_requested_bidirectional_room":
        best_score.first==required_count?"earliest_priority_measurement_room_on_demonstrated_path":"partial_measurement_room_on_demonstrated_path";
    return out;
}

inline DemonstratedCenterPlan plan_demonstrated_measurement_return(const DemonstratedCenterPlan& entry,
    const std::vector<double>& fresh,const std::vector<double>& endpoint,std::size_t endpoint_source_index,
    const std::vector<double>& entry_tolerance,const std::vector<double>& observed_position_step_rad={}) {
    DemonstratedCenterPlan out;const auto n=fresh.size();
    const auto reject=[&](const char* why){out.valid=false;out.reason=why;return out;};
    if(!entry.valid || !n || endpoint.size()!=n || entry_tolerance.size()!=n ||
       entry.demonstrated_route.empty() || entry.demonstrated_route.size()!=entry.demonstrated_source_indices.size() ||
       entry.demonstrated_lower.size()!=n || entry.demonstrated_upper.size()!=n ||
       !measurement_path_detail::finite(fresh) || !measurement_path_detail::finite(endpoint))
        return reject("measurement_return_original_route_unavailable");
    if(entry.demonstrated_route.front()!=endpoint || entry.demonstrated_source_indices.front()!=endpoint_source_index)
        return reject("measurement_return_endpoint_does_not_match_original_demonstration");
    std::vector<std::vector<double>> reversed(entry.demonstrated_route.rbegin(),entry.demonstrated_route.rend());
    std::vector<std::size_t> source(entry.demonstrated_source_indices.rbegin(),entry.demonstrated_source_indices.rend());
    std::vector<std::size_t> preserve_bridge;
    for(std::size_t k=0;k<reversed.size();++k)
        if(source[k]==std::numeric_limits<std::size_t>::max() || reversed[k].size()!=n || !measurement_path_detail::finite(reversed[k]))
            return reject("measurement_return_original_route_invalid");
    if(reversed.size()==1) {
        for(std::size_t i=0;i<n;++i)if(!std::isfinite(entry_tolerance[i]) || entry_tolerance[i]<=0 ||
            std::abs(fresh[i]-endpoint[i])>entry_tolerance[i])return reject("measurement_return_outside_existing_alignment");
        out=entry;out.route={fresh};out.source_indices={std::numeric_limits<std::size_t>::max()};
        if(fresh!=endpoint){out.route.push_back(endpoint);out.source_indices.push_back(endpoint_source_index);}
    } else {
        DemonstratedCenterRequest req;req.fresh_position=fresh;req.entry_tolerance=entry_tolerance;req.desired_room.assign(n,0);
        req.observed_position_step_rad=observed_position_step_rad.empty()?entry.observed_position_step_rad:observed_position_step_rad;
        req.priority_axes.assign(n,false);req.safety_lower.assign(n,-std::numeric_limits<double>::infinity());req.safety_upper.assign(n,std::numeric_limits<double>::infinity());
        const auto projection=plan_demonstrated_measurement_center(reversed,req);
        if(!projection.valid)return reject("measurement_return_no_nearby_original_path_projection");
        out=entry;out.route.clear();out.source_indices.clear();
        const auto append=[&](const std::vector<double>& q,std::size_t index){if(out.route.empty() || out.route.back()!=q){out.route.push_back(q);out.source_indices.push_back(index);}};
        append(fresh,std::numeric_limits<std::size_t>::max());
        append(projection.center,source[projection.entry_source_index]);
        if(fresh!=projection.center)preserve_bridge.push_back(1);
        for(std::size_t k=projection.entry_source_index+1;k<reversed.size();++k)append(reversed[k],source[k]);
        out.entry_source_index=source[projection.entry_source_index];out.entry_fraction=projection.entry_fraction;
    }
    out.center=endpoint;out.target_source_index=endpoint_source_index;out.reverse=!entry.reverse;
    const auto& resolution=observed_position_step_rad.empty()?entry.observed_position_step_rad:observed_position_step_rad;
    const auto simplified=simplify_demonstrated_route(out.route,out.source_indices,resolution,preserve_bridge);
    if(!simplified.valid)return reject("measurement_return_position_resolution_invalid");
    out.route_original_nodes=simplified.original_nodes;out.route=simplified.route;out.source_indices=simplified.source_indices;
    out.observed_position_step_rad=resolution;out.route_simplification_tolerance_rad=simplified.tolerance_rad;
    out.available_room.resize(n);out.room_satisfied.assign(n,true);out.insufficient_axes.clear();
    for(std::size_t i=0;i<n;++i)out.available_room[i]=std::max(0.0,std::min(endpoint[i]-entry.demonstrated_lower[i],entry.demonstrated_upper[i]-endpoint[i]));
    out.valid=true;out.migration_required=out.route.size()>1;out.reason="fresh_projection_then_reverse_original_demonstrated_route";
    out.entry_scope="fresh_feedback_bridge_with_existing_alignment_tolerance_then_reverse_original_demonstrated_path";
    return out;
}

struct PathProgress {std::size_t segment{0};double distance{0},speed{0};};
// A gain handover must not suddenly consume the braking distance already
// committed by a moving reference. Defer only genuinely tighter leads until
// rest; the caller still checks the stopped reference error against the new
// hard lead and its uncertainty reserve before applying any gain change.
inline bool measurement_path_lead_change_allowed(const PathProgress& progress,
    const std::vector<double>& previous_lead,const std::vector<double>& next_lead) {
    if(previous_lead.empty() || previous_lead.size()!=next_lead.size() || !std::isfinite(progress.speed) || progress.speed<0)return false;
    for(std::size_t i=0;i<previous_lead.size();++i) {
        if(!std::isfinite(previous_lead[i]) || previous_lead[i]<=0 || !std::isfinite(next_lead[i]) || next_lead[i]<=0)return false;
        if(next_lead[i]<previous_lead[i]-1e-12 && progress.speed>1e-7)return false;
    }
    return true;
}
struct PathAdvanceResult {
    bool valid{false},complete{false},feedback_limited{false};
    PathProgress progress;
    std::vector<double> position,velocity;
    std::string reason;
};
// A common scalar distance preserves each complete path segment. Corners are
// reached at rest; their direction changes cannot introduce instantaneous joint
// velocity jumps. The acceleration-limited stop is planned before the feedback
// lead is consumed, rather than clipping individual joint references afterward.
inline PathAdvanceResult advance_demonstrated_path(const std::vector<std::vector<double>>& route,
    PathProgress state,const std::vector<double>& actual,const std::vector<double>& lead,
    const std::vector<double>& max_velocity,const std::vector<double>& max_acceleration,double dt,
    const std::vector<double>& position_uncertainty={}) {
    PathAdvanceResult out;out.progress=state;const auto n=actual.size();
    const auto reject=[&](const char* why){out.reason=why;return out;};
    if(!n || route.empty() || lead.size()!=n || max_velocity.size()!=n || max_acceleration.size()!=n ||
       (!position_uncertainty.empty() && position_uncertainty.size()!=n) ||
       !std::isfinite(dt) || dt<=0 || !std::isfinite(state.distance) || state.distance<0 || !std::isfinite(state.speed) || state.speed<0)
        return reject("path_progress_input_invalid");
    for(const auto& q:route)if(q.size()!=n || !measurement_path_detail::finite(q))return reject("path_progress_mapping_invalid");
    for(std::size_t i=0;i<n;++i)if(!std::isfinite(actual[i]) || !std::isfinite(lead[i]) || lead[i]<=0 ||
        !std::isfinite(max_velocity[i]) || max_velocity[i]<=0 || !std::isfinite(max_acceleration[i]) || max_acceleration[i]<=0)
        return reject("path_progress_feedback_or_limits_invalid");
    for(std::size_t i=0;i<position_uncertainty.size();++i)
        if(!std::isfinite(position_uncertainty[i]) || position_uncertainty[i]<0 || position_uncertainty[i]>=lead[i])
            return reject("path_position_uncertainty_consumes_feedback_lead");
    if(state.segment>=route.size()-1) {
        // Reaching a reference endpoint does not suspend the raw-feedback
        // lead during the caller's executed zero-velocity dwell.
        for(std::size_t i=0;i<n;++i)if(std::abs(route.back()[i]-actual[i])>lead[i]+1e-12)
            return reject("path_reference_outside_feedback_lead");
        out.valid=out.complete=true;out.position=route.back();out.velocity.assign(n,0);
        out.progress={route.size()-1,0,0};out.reason="demonstrated_path_target_reached";return out;
    }
    const auto& a=route[state.segment];const auto& b=route[state.segment+1];double length=0;
    for(std::size_t i=0;i<n;++i)length=std::max(length,std::abs(b[i]-a[i]));
    if(length==0) {
        for(std::size_t i=0;i<n;++i)if(std::abs(b[i]-actual[i])>lead[i]+1e-12)
            return reject("path_reference_outside_feedback_lead");
        out.valid=true;out.progress={state.segment+1,0,0};out.position=b;out.velocity.assign(n,0);
        out.complete=out.progress.segment==route.size()-1;out.reason="demonstrated_path_stationary_node";return out;
    }
    if(state.distance>length || state.speed<0)return reject("path_progress_state_outside_segment");
    double speed_limit=std::numeric_limits<double>::infinity(),acceleration=std::numeric_limits<double>::infinity(),stop=length,hard_stop=length;
    std::vector<double> direction(n);
    for(std::size_t i=0;i<n;++i) {
        direction[i]=(b[i]-a[i])/length;
        const double q=state.distance==length?b[i]:a[i]+direction[i]*state.distance;
        if(std::abs(q-actual[i])>lead[i]+1e-12)return reject("path_reference_outside_feedback_lead");
        if(direction[i]!=0) {
            speed_limit=std::min(speed_limit,max_velocity[i]/std::abs(direction[i]));
            acceleration=std::min(acceleration,max_acceleration[i]/std::abs(direction[i]));
            const double edge=actual[i]+(direction[i]>0?lead[i]:-lead[i]);
            hard_stop=std::min(hard_stop,(edge-a[i])/direction[i]);
            // Reserve encoder/noise uncertainty before consuming the hard
            // feedback lead. A later code transition may consume this reserve
            // while braking; it never enlarges the original hard limit.
            const double uncertainty=position_uncertainty.empty()?0:position_uncertainty[i];
            const double planned_edge=actual[i]+(direction[i]>0?1:-1)*(lead[i]-uncertainty);
            stop=std::min(stop,(planned_edge-a[i])/direction[i]);
        }
    }
    const double room=std::max(0.0,stop-state.distance);out.feedback_limited=stop<length;
    const double hard_room=std::max(0.0,hard_stop-state.distance);
    if(state.speed*state.speed>2*acceleration*hard_room+1e-12)return reject("path_feedback_lead_below_required_braking_distance");
    const double minimum_speed=std::max(0.0,state.speed-acceleration*dt);
    const double adt=acceleration*dt;
    const double discriminant=adt*adt-4*(adt*state.speed-2*acceleration*room);
    const double safe_speed=discriminant>=0?std::max(0.0,.5*(-adt+std::sqrt(discriminant))):0;
    const double next_speed=std::max(minimum_speed,std::min({speed_limit,state.speed+adt,safe_speed}));
    double distance=.5*(state.speed+next_speed)*dt;
    if(next_speed==0 && state.speed<adt)distance=state.speed*state.speed/(2*acceleration);
    if(distance+next_speed*next_speed/(2*acceleration)>hard_room+1e-12)
        return reject("path_feedback_lead_cannot_be_retained_with_acceleration_limit");
    out.progress.distance=std::min(length,state.distance+distance);out.progress.speed=next_speed;
    // The exact terminal coordinate is used only after the scalar motion has
    // stopped. It is not an angle clamp or a replacement feedback measurement.
    if(length-out.progress.distance<=1e-12 && next_speed<=1e-10){out.progress={state.segment+1,0,0};out.position=b;out.velocity.assign(n,0);}
    else {
        out.position.resize(n);out.velocity.resize(n);
        for(std::size_t i=0;i<n;++i){out.position[i]=a[i]+direction[i]*out.progress.distance;out.velocity[i]=direction[i]*next_speed;}
    }
    out.valid=true;out.complete=out.progress.segment==route.size()-1;
    out.reason=out.complete?"demonstrated_path_target_reached":out.feedback_limited?"feedback_bounded_demonstrated_path_progress":"demonstrated_path_progress";
    return out;
}
inline double demonstrated_path_transition_seconds(const std::vector<std::vector<double>>& route,
    const std::vector<double>& max_velocity,const std::vector<double>& max_acceleration) {
    if(route.empty() || max_velocity.empty() || max_velocity.size()!=max_acceleration.size())return std::numeric_limits<double>::infinity();
    double seconds=0;
    for(std::size_t k=1;k<route.size();++k) {
        if(route[k].size()!=max_velocity.size() || route[k-1].size()!=max_velocity.size())return std::numeric_limits<double>::infinity();
        double length=0;for(std::size_t i=0;i<max_velocity.size();++i)length=std::max(length,std::abs(route[k][i]-route[k-1][i]));
        if(length==0)continue;
        double v=std::numeric_limits<double>::infinity(),a=v;
        for(std::size_t i=0;i<max_velocity.size();++i)if(route[k][i]!=route[k-1][i]) {
            const double direction=std::abs(route[k][i]-route[k-1][i])/length;
            if(!std::isfinite(max_velocity[i]) || max_velocity[i]<=0 || !std::isfinite(max_acceleration[i]) || max_acceleration[i]<=0)return std::numeric_limits<double>::infinity();
            v=std::min(v,max_velocity[i]/direction);a=std::min(a,max_acceleration[i]/direction);
        }
        seconds+=length<=v*v/a?2*std::sqrt(length/a):length/v+v/a;
    }
    return seconds;
}
} // namespace serial_arm::servo_tuning
