#pragma once

#include "serial_arm/robot.hpp"

#include <algorithm>
#include <cmath>
#include <deque>
#include <limits>
#include <string>

namespace serial_arm {

struct JointMotionStatus {
    std::uint64_t id{0};
    std::string state{"idle"};
    JointVector target,position_tolerance,velocity_tolerance;
    std::vector<std::string> units;
    bool reference_complete{false};
    bool target_reached{false};
    JointVector position_error;
    double settled_s{0.0};
    std::string reason;
};

// Tracks a point-to-point request using the existing reference generator and
// measured cycles; it never generates commands or owns a control loop
class JointMotionTracker {
public:
    std::uint64_t begin(const JointVector& target,const std::vector<std::string>& joint_types) {
        status_=JointMotionStatus{};status_.id=++sequence_;status_.state="running";status_.target=target;
        for(std::size_t i=0;i<target.size();++i) {
            const bool linear=i<joint_types.size()&&joint_types[i]=="prismatic";
            status_.units.push_back(linear?"m":"rad");
            status_.position_tolerance.push_back(linear?0.0001:0.001);
            status_.velocity_tolerance.push_back(linear?0.001:0.01);
        }
        last_measurement_ns_=0;last_stamps_.clear();positions_.assign(target.size(),{});was_settled_=false;return status_.id;
    }
    void reference_complete(bool complete) {if(status_.state=="running")status_.reference_complete=complete;}
    void finish(const std::string& state,const std::string& reason) {
        if(status_.state!="running")return;
        status_.state=state;status_.reason=reason;status_.settled_s=0.0;
    }
    void update(const RobotCycleOutput& cycle,double max_age_ms) {
        if(status_.state!="running")return;
        const auto n=status_.target.size();
        const auto& stamps=cycle.actuator_state.feedback_received_ns;
        const auto& timing=cycle.actuator_state.feedback_timing_valid;
        const bool fresh=std::isfinite(cycle.feedback_age_ms)&&
            cycle.feedback_age_ms>=0&&cycle.feedback_age_ms<=max_age_ms&&stamps.size()==n&&timing.size()==n&&
            std::all_of(timing.begin(),timing.end(),[](auto valid){return valid!=0;})&&
            cycle.joint_state.pos.size()==n&&cycle.joint_state.vel.size()==n;
        if(!fresh){reset_measurements();return;}
        if(last_stamps_.size()==n)for(std::size_t i=0;i<n;++i)if(stamps[i]<=last_stamps_[i])return;
        const auto stamp=*std::min_element(stamps.begin(),stamps.end());
        if(!stamp||stamp<=last_measurement_ns_)return;
        const double dt=last_measurement_ns_?static_cast<double>(stamp-last_measurement_ns_)/1e9:0.0;
        last_measurement_ns_=stamp;last_stamps_=stamps;
        if(dt>max_age_ms/1000.0){for(auto& history:positions_)history.clear();was_settled_=false;status_.settled_s=0;}
        status_.position_error.resize(n);
        bool settled=status_.reference_complete,at_target=true;
        for(std::size_t i=0;i<n;++i) {
            const double q=cycle.joint_state.pos[i];
            if(!std::isfinite(q)||!std::isfinite(cycle.joint_state.vel[i])){reset_measurements();return;}
            status_.position_error[i]=q-status_.target[i];
            at_target=at_target&&std::abs(status_.position_error[i])<=status_.position_tolerance[i];
            auto& history=positions_[i];history.push_back({stamps[i],q});
            constexpr std::uint64_t window_ns=250000000;
            while(history.size()>2 && stamps[i]-history[1].stamp>=window_ns)history.pop_front();
            const double span_s=static_cast<double>(stamps[i]-history.front().stamp)/1e9;
            if(span_s<.25){settled=false;continue;}
            double sum_t=0,sum_q=0,sum_tt=0,sum_tq=0,lo=q,hi=q;
            for(const auto& sample:history) {
                const double t=static_cast<double>(sample.stamp-history.front().stamp)/1e9;
                sum_t+=t;sum_q+=sample.position;sum_tt+=t*t;sum_tq+=t*sample.position;
                lo=std::min(lo,sample.position);hi=std::max(hi,sample.position);
            }
            const double count=static_cast<double>(history.size()),denominator=count*sum_tt-sum_t*sum_t;
            const double speed=denominator>0?(count*sum_tq-sum_t*sum_q)/denominator:std::numeric_limits<double>::infinity();
            // Returned motor velocity can retain a code-sized offset while
            // position is stationary. Observe actual travel over fresh receive
            // timestamps; also bound its span so oscillations cannot cancel
            // into a zero regression slope. This only completes the UI request;
            // raw velocity and the command/reference generator are unchanged.
            settled=settled&&std::abs(speed)<=status_.velocity_tolerance[i]&&
                hi-lo<=std::max(2*status_.position_tolerance[i],status_.velocity_tolerance[i]*span_s);
        }
        if(!settled){status_.settled_s=0.0;was_settled_=false;return;}
        if(was_settled_&&dt>0&&dt<=max_age_ms/1000.0)status_.settled_s+=dt;
        else status_.settled_s=0.0;
        was_settled_=true;
        if(status_.settled_s>=0.15) {
            status_.state="completed";status_.target_reached=at_target;
            status_.reason=at_target?"target_reached":"settled_with_position_offset";
        }
    }
    const JointMotionStatus& status() const {return status_;}
private:
    struct PositionSample {std::uint64_t stamp;double position;};
    void reset_measurements() {
        status_.settled_s=0;status_.position_error.clear();last_measurement_ns_=0;last_stamps_.clear();
        for(auto& history:positions_)history.clear();
        was_settled_=false;
    }
    JointMotionStatus status_;
    std::uint64_t sequence_{0},last_measurement_ns_{0};
    std::vector<std::uint64_t> last_stamps_;
    std::vector<std::deque<PositionSample>> positions_;
    bool was_settled_{false};
};

struct TeachReplayPlan {
    std::string direction;
    int cycles{1},total_passes{1};
    bool first_reverse{false};
};
inline TeachReplayPlan teach_replay_plan(const std::string& direction,int cycles) {
    const bool cyclic=direction=="cycle"||direction=="contact_cycle";
    return {direction,cyclic?cycles:1,cyclic?2*cycles:1,
        direction=="reverse"||direction=="return"||direction=="contact_cycle"};
}
} // namespace serial_arm
