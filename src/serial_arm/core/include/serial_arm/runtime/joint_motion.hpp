#pragma once

#include "serial_arm/robot.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace serial_arm {

struct JointMotionStatus {
    std::uint64_t id{0};
    std::string state{"idle"};
    JointVector target,position_tolerance,velocity_tolerance;
    std::vector<std::string> units;
    bool reference_complete{false};
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
        last_measurement_ns_=0;last_stamps_.clear();was_settled_=false;return status_.id;
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
        if(!fresh){status_.settled_s=0.0;last_measurement_ns_=0;last_stamps_.clear();was_settled_=false;return;}
        if(last_stamps_.size()==n)for(std::size_t i=0;i<n;++i)if(stamps[i]<=last_stamps_[i])return;
        const auto stamp=*std::min_element(stamps.begin(),stamps.end());
        if(!stamp||stamp<=last_measurement_ns_)return;
        const double dt=last_measurement_ns_?static_cast<double>(stamp-last_measurement_ns_)/1e9:0.0;
        last_measurement_ns_=stamp;last_stamps_=stamps;
        bool settled=status_.reference_complete;
        for(std::size_t i=0;i<n;++i) {
            settled=settled&&std::isfinite(cycle.joint_state.pos[i])&&std::isfinite(cycle.joint_state.vel[i])&&
                std::abs(cycle.joint_state.pos[i]-status_.target[i])<=status_.position_tolerance[i]&&
                std::abs(cycle.joint_state.vel[i])<=status_.velocity_tolerance[i];
        }
        if(!settled){status_.settled_s=0.0;was_settled_=false;return;}
        if(was_settled_&&dt>0&&dt<=max_age_ms/1000.0)status_.settled_s+=dt;
        else status_.settled_s=0.0;
        was_settled_=true;
        if(status_.settled_s>=0.15)status_.state="completed";
    }
    const JointMotionStatus& status() const {return status_;}
private:
    JointMotionStatus status_;
    std::uint64_t sequence_{0},last_measurement_ns_{0};
    std::vector<std::uint64_t> last_stamps_;
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
