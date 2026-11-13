#include "serial_arm/runtime/joint_motion.hpp"
#include <iostream>
#include <stdexcept>

using namespace serial_arm;
namespace {
void require(bool value,const char* message){if(!value)throw std::runtime_error(message);}
RobotCycleOutput feedback(std::size_t n,double t,double q,double raw_velocity=.04) {
    RobotCycleOutput cycle;
    cycle.feedback_age_ms=2;
    cycle.joint_state.pos.assign(n,q);cycle.joint_state.vel.assign(n,raw_velocity);
    cycle.actuator_state.feedback_received_ns.assign(n,1000000000ULL+static_cast<std::uint64_t>(t*1e9));
    cycle.actuator_state.feedback_timing_valid.assign(n,1);
    return cycle;
}
void stationary_quantized_feedback_and_offsets() {
    for(const std::size_t n:{1U,3U,7U}) {
        JointMotionTracker arrived;arrived.begin(JointVector(n,0),{});arrived.reference_complete(true);
        for(int k=0;k<120;++k)arrived.update(feedback(n,.005*k,k%3==0?.000381:0),100);
        require(arrived.status().state=="completed" && arrived.status().target_reached,
            "stationary quantized position stayed running because returned velocity has a code offset");
        JointMotionTracker offset;offset.begin(JointVector(n,0),{});offset.reference_complete(true);
        for(int k=0;k<120;++k)offset.update(feedback(n,.005*k,.03),100);
        require(offset.status().state=="completed" && !offset.status().target_reached &&
            offset.status().reason=="settled_with_position_offset" && offset.status().position_error==JointVector(n,.03),
            "a stationary offset either blocked request completion or falsely claimed target arrival");
    }
    JointMotionTracker linear;linear.begin({0},{"prismatic"});linear.reference_complete(true);
    for(int k=0;k<120;++k)linear.update(feedback(1,.005*k,.0002),100);
    require(linear.status().state=="completed" && !linear.status().target_reached && linear.status().units[0]=="m",
        "prismatic units or arrival tolerance were replaced by rotational thresholds");
}
void movement_and_reference_are_not_completed_by_a_timer() {
    JointMotionTracker moving;moving.begin({1},{});moving.reference_complete(true);
    JointMotionTracker oscillating;oscillating.begin({0},{});oscillating.reference_complete(true);
    JointMotionTracker unsent;unsent.begin({0},{});
    for(int k=0;k<400;++k) {
        const double t=.005*k;
        moving.update(feedback(1,t,.08*t,0),100);
        oscillating.update(feedback(1,t,.02*std::sin(2*3.141592653589793*4*t),0),100);
        unsent.update(feedback(1,t,0,0),100);
    }
    require(moving.status().state=="running","actual travel was called complete with a quiet raw velocity code");
    require(oscillating.status().state=="running","alternating travel canceled into a false zero-speed completion");
    require(unsent.status().state=="running","a quiet arm completed before its reference generator finished");
    for(int k=400;k<520;++k)moving.update(feedback(1,.005*k,.16,0),100);
    require(moving.status().state=="completed" && !moving.status().target_reached,
        "actual motion did not finish after measured position became stable");
}
void freshness_terminal_events_and_gaps() {
    JointMotionTracker duplicate;duplicate.begin({0},{});duplicate.reference_complete(true);
    for(int k=0;k<500;++k)duplicate.update(feedback(1,0,0),100);
    require(duplicate.status().state=="running","duplicate packets manufactured a settling duration");
    JointMotionTracker stale;stale.begin({0,0},{});stale.reference_complete(true);
    for(int k=0;k<120;++k){auto c=feedback(2,.005*k,0);c.actuator_state.feedback_timing_valid[1]=0;stale.update(c,100);}
    require(stale.status().state=="running" && stale.status().position_error.empty(),"invalid per-axis timing manufactured completion evidence");
    JointMotionTracker gap;gap.begin({0},{});gap.reference_complete(true);
    for(int k=0;k<45;++k)gap.update(feedback(1,.005*k,0),100);
    for(int k=100;k<150;++k)gap.update(feedback(1,.005*k,0),100);
    require(gap.status().state=="running","a reception gap reused earlier quiet time");
    for(int k=150;k<200;++k)gap.update(feedback(1,.005*k,0),100);
    require(gap.status().state=="completed","fresh continuous feedback did not recover after a gap");
    for(const auto* terminal:{"failed","interrupted"}) {
        JointMotionTracker stopped;stopped.begin({0},{});stopped.reference_complete(true);stopped.finish(terminal,"actual terminal event");
        for(int k=0;k<120;++k)stopped.update(feedback(1,.005*k,0),100);
        require(stopped.status().state==terminal && !stopped.status().target_reached,"quiet feedback overwrote a real fault or stop");
    }
}
}
int main(){try {
    stationary_quantized_feedback_and_offsets();movement_and_reference_are_not_completed_by_a_timer();freshness_terminal_events_and_gaps();
    std::cout<<"PASS joint motion: fresh position-window settling, truthful arrival/offset, 1/3/7 axes, prismatic, ongoing motion, oscillation, gaps, duplicates and actual terminal events\n";
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
