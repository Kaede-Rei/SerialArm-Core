#include "serial_arm/dynamics/gain_taught_measurement_schedule.hpp"

#include <iostream>
#include <stdexcept>

using namespace serial_arm::servo_tuning;
namespace {
void check(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
void close(double actual,double expected,const char* message,double tolerance=1e-10) {
    check(std::abs(actual-expected)<=tolerance,message);
}

void interior_selection() {
    TaughtMeasurementRoomRequest request;
    request.required_room_rad={.1,.1,.02};request.observed_position_step_rad={.001,.001,.001};
    request.preferred_source_indices={1,3};
    const std::vector<std::vector<double>> path{{0,0,.3},{.2,.8,.3},{0,0,.3},{.8,.2,.3},{1,1,.3}};
    const auto schedule=plan_taught_measurement_schedule(path,request);
    check(schedule.valid && schedule.has_anchor,"valid demonstrated interior was not scheduled");
    check(schedule.common_anchor.source_index==1 && schedule.common_anchor.preferred,
        "equally good remapped existing pose targets must be preferred deterministically");
    check(schedule.common_anchor.measurable_axis_count==2 && !schedule.axis_anchors[2].available,
        "a stationary axis must not invalidate measurable axes or invent its own motion evidence");
    check(schedule.common_anchor.source_index!=2,"a later exact endpoint/zero repeat is not an interior equilibrium");
    close(schedule.common_anchor.minimum_normalized_room,.4,"room score must normalize each axis by its own demonstrated range");

    request.preferred_source_indices={3};
    const auto preferred=plan_taught_measurement_schedule(path,request);
    check(preferred.common_anchor.source_index==3,"same-quality existing target preference was ignored");
    const std::vector<std::vector<double>> separate{{0,0},{.5,0},{1,.1},{1,.5},{1,1}};
    request.required_room_rad={.2,.2};request.observed_position_step_rad={.001,.001};request.preferred_source_indices={};
    const auto axes=plan_taught_measurement_schedule(separate,request);
    check(axes.valid && axes.axis_anchors[0].source_index==1 && axes.axis_anchors[1].source_index==3,
        "different axes need their own internal equilibria when no single pose measures both");
    check(axes.common_anchor.measurable_axis_count==1,"unmeasurable axes were silently counted as covered");

    // Zero is a valid interior angle on a signed range. Only an endpoint repeat
    // is excluded; no joint name, index, or numerical zero is special-cased.
    request.required_room_rad={.2};request.observed_position_step_rad={};
    const auto signed_path=plan_taught_measurement_schedule({{-1},{0},{1}},request);
    check(signed_path.has_anchor && signed_path.common_anchor.position[0]==0,"legitimate signed-range center was excluded as zero");
}

void actual_feedback_owns_room() {
    TaughtMeasurementRoomRequest request;
    request.required_room_rad={.03,.03};request.observed_position_step_rad={.002,.002};
    request.stationary_noise_rms_rad={.001,.001};
    const auto schedule=plan_taught_measurement_schedule({{0,0},{.5,.5},{1,1}},request);
    check(schedule.valid && schedule.has_anchor,"fixture has no planned interior");
    const auto actual=evaluate_taught_measurement_room({.08,.93},schedule,request);
    check(actual.valid && actual.bidirectional_measurable[0] && actual.bidirectional_measurable[1],
        "actual feedback with measurable room was rejected for differing from the planned center");
    close(actual.negative_room_rad[0],.08,"actual center was projected onto the plan");
    const auto near_stop=evaluate_taught_measurement_room({0,.5},schedule,request);
    check(near_stop.valid && !near_stop.bidirectional_measurable[0] && near_stop.positive_measurable[0] &&
          near_stop.bidirectional_measurable[1] && near_stop.any_measurable,
        "one axis at a mechanical endpoint must skip its local test while other axes remain usable");
    const auto beyond=evaluate_taught_measurement_room({-.01,.5},schedule,request);
    check(beyond.valid && !beyond.positive_measurable[0] && beyond.positive_room_rad[0]==0,
        "feedback outside the demonstrated envelope was clamped into an invented experiment");
    request.required_room_rad={.001,.001};request.observed_position_step_rad={.02,.002};
    request.stationary_noise_rms_rad={0,.02};
    const auto uncertain=evaluate_taught_measurement_room({.05,.9},schedule,request);
    check(uncertain.valid && !uncertain.any_measurable && uncertain.required_room_rad[0]==.06 &&
          uncertain.required_room_rad[1]==.12,"encoder resolution and noise must set actual measurability, not fabricated successful evidence");
}

void arbitrary_joint_count_and_missing_room() {
    for(const auto joints:{1U,4U,7U}) {
        std::vector<std::vector<double>> path(5,std::vector<double>(joints));
        TaughtMeasurementRoomRequest request;request.required_room_rad.resize(joints);
        for(std::size_t j=0;j<joints;++j) {
            const double scale=.1*(j+1);request.required_room_rad[j]=.05*scale;
            for(std::size_t k=0;k<path.size();++k)path[k][j]=scale*k;
        }
        const auto schedule=plan_taught_measurement_schedule(path,request);
        check(schedule.valid && schedule.common_anchor.source_index==2 && schedule.common_anchor.measurable_axis_count==joints,
            "schedule depends on a particular joint count, joint identity, or inertia scale");
    }
    TaughtMeasurementRoomRequest request;request.required_room_rad={.1};
    const auto flat=plan_taught_measurement_schedule({{0},{0},{0}},request);
    check(flat.valid && !flat.has_anchor && !flat.axis_anchors[0].available,
        "no internal room must preserve path executability without a false measurement");
    const auto only_endpoints=plan_taught_measurement_schedule({{0},{1}},request);
    check(only_endpoints.valid && !only_endpoints.has_anchor,"endpoints became internal scheduled anchors");
    const auto short_range=plan_taught_measurement_schedule({{0},{.01},{.02}},request);
    check(short_range.valid && !short_range.has_anchor,"insufficient demonstration was converted to a successful probe");
    request.preferred_source_indices={3};
    check(!plan_taught_measurement_schedule({{0},{.5},{1}},request).valid,"unmapped preferred indices were accepted");
    request.preferred_source_indices={};request.required_room_rad={};
    check(!plan_taught_measurement_schedule({{0},{.5},{1}},request).valid,"wrong axis mapping was accepted");
}

void split_cubic_clock() {
    const auto warp=make_taught_measurement_time_warp(10,4,2,.6);
    check(warp.valid,"ordinary interior time warp failed");
    close(warp.inserted_time_s,2.6,"actual transition and dwell cost was omitted");
    close(warp.wall_duration_s,12.6,"time warp cost must be included in elapsed budget");
    double previous=0;
    for(unsigned k=0;k<=12600;++k) {
        const double wall=k*.001;
        const auto clock=sample_taught_measurement_time_warp(warp,wall);
        check(clock.valid && clock.source_time_s>=previous-1e-12 && clock.source_time_s>=0 && clock.source_time_s<=10,
            "measurement time warp reverses, skips, or leaves the original demonstration");
        check(clock.rate>=0 && clock.rate<=1 && std::abs(clock.rate_acceleration_s_inv)<=2/.6+1e-10,
            "split cubic rate or phase acceleration exceeded its analytic bound");
        if(clock.at_anchor)check(clock.source_time_s==4 && clock.rate==0 && clock.rate_acceleration_s_inv==0,
            "measurement dwell did not freeze source q and qdot at the same demonstrated anchor");
        if(wall>warp.braking_wall_start_s+1e-4 && wall<warp.normal_wall_resume_s-1e-4) {
            const auto before=sample_taught_measurement_time_warp(warp,wall-1e-5);
            const auto after=sample_taught_measurement_time_warp(warp,wall+1e-5);
            close((after.source_time_s-before.source_time_s)/2e-5,clock.rate,
                "recorded velocity rate is not the derivative of actual reference phase",2e-8);
        }
        previous=clock.source_time_s;
    }
    for(const double join:{warp.braking_wall_start_s,warp.braking_wall_start_s+.3,warp.dwell_wall_start_s,
                           warp.resume_wall_start_s,warp.resume_wall_start_s+.3,warp.normal_wall_resume_s}) {
        const auto left=sample_taught_measurement_time_warp(warp,join-1e-7);
        const auto right=sample_taught_measurement_time_warp(warp,join+1e-7);
        close(left.source_time_s,right.source_time_s,"split cubic source position is discontinuous",3e-7);
        close(left.rate,right.rate,"split cubic source velocity is discontinuous",7e-7);
        close(left.rate_acceleration_s_inv,right.rate_acceleration_s_inv,"split cubic source acceleration is discontinuous",3e-6);
    }
    const auto normal=sample_taught_measurement_time_warp(warp,2);
    const auto resumed=sample_taught_measurement_time_warp(warp,10);
    check(normal.source_time_s==2 && normal.rate==1 && resumed.rate==1,
        "ordinary taught-path motion must remain 1x outside its planned measurement transition");
    close(resumed.source_time_s,7.4,"source phase did not resume after the actual inserted cost");
    const auto complete=advance_taught_measurement_time_warp(warp,12,.7);
    check(complete.valid && complete.complete && complete.source_time_s==10 && complete.wall_time_s==12.6,
        "large control step failed to finish at original path endpoint");
    check(!make_taught_measurement_time_warp(10,.1,1,.6).valid,
        "near endpoint anchor cannot bypass smooth braking distance");
    check(!make_taught_measurement_time_warp(10,4,1,.1).valid &&
          !advance_taught_measurement_time_warp(warp,1,-.1).valid,
        "unsupported transition duration or backward clock was accepted");
}
void response_priority_escapes_long_plateau() {
    for(std::size_t joints:{1,3,7}) {
        constexpr double dt=.02;const std::size_t samples=1201,focus=joints/2;
        std::vector<std::vector<double>> path(samples,std::vector<double>(joints));
        TaughtMeasurementRoomRequest request;request.required_room_rad.assign(joints,.01);
        request.observed_position_step_rad.assign(joints,.0004);request.stationary_noise_rms_rad.assign(joints,.0001);
        request.preferred_source_indices={600};
        std::vector<double> weights(joints,1);weights[focus]=100;
        for(std::size_t k=0;k<samples;++k)for(std::size_t j=0;j<joints;++j) {
            const double t=k*dt,scale=1+.2*j;
            path[k][j]=scale*(j==focus?(t<4?.125*t:t<=20?.5:.5+.125*(t-20)):t/24);
        }
        const auto geometry=plan_taught_measurement_schedule(path,request);
        check(geometry.has_anchor && geometry.common_anchor.source_index==600,
            "long-plateau fixture must reproduce the geometric room midpoint preference");
        const auto response=plan_taught_response_measurement_schedule(path,request,dt,weights);
        check(response.valid && response.has_anchor && response.common_anchor.measurable_axis_count==joints &&
            response.common_anchor.source_index!=600 && response.common_anchor.nearby_reference_span_rad[focus]>.3,
            "a weak axis must move measurement beside its naturally moving taught part while preserving measurable room coverage");
        check(response.common_anchor.source_index*dt>=1 &&
            (samples-1-response.common_anchor.source_index)*dt>=1,
            "response-weighted selection must leave source-clock braking and resume room");
        auto shuffled=path;auto reverse_weights=weights;auto reverse_request=request;
        for(auto& q:shuffled)std::reverse(q.begin(),q.end());
        std::reverse(reverse_weights.begin(),reverse_weights.end());
        std::reverse(reverse_request.required_room_rad.begin(),reverse_request.required_room_rad.end());
        std::reverse(reverse_request.observed_position_step_rad.begin(),reverse_request.observed_position_step_rad.end());
        std::reverse(reverse_request.stationary_noise_rms_rad.begin(),reverse_request.stationary_noise_rms_rad.end());
        const auto permuted=plan_taught_response_measurement_schedule(shuffled,reverse_request,dt,reverse_weights);
        check(permuted.common_anchor.source_index==response.common_anchor.source_index,
            "response anchor selection must follow the physical evidence under arbitrary joint permutation");
        const auto fallback=plan_taught_response_measurement_schedule(path,request,dt,std::vector<double>(joints,0));
        check(fallback.common_anchor.source_index==geometry.common_anchor.source_index,
            "no resolved response priority must retain the existing geometry behavior");
    }
    TaughtMeasurementRoomRequest request;request.required_room_rad={.1,.1};
    request.observed_position_step_rad={.001,.001};
    // Motion priority cannot trade away an otherwise available measurement
    // axis or turn an endpoint into a perturbation equilibrium.
    const std::vector<std::vector<double>> path{{0,0},{.2,0},{.4,0},{.5,.5},{.6,1},{.8,1},{1,1}};
    const auto covered=plan_taught_response_measurement_schedule(path,request,1,{100,1},.5);
    check(covered.has_anchor && covered.common_anchor.source_index==3 && covered.common_anchor.measurable_axis_count==2,
        "higher reference motion cannot replace a geometry-eligible two-axis anchor with a one-axis endpoint-side anchor");
    check(!plan_taught_response_measurement_schedule(path,request,0,{1,1}).valid &&
        !plan_taught_response_measurement_schedule(path,request,1,{1}).valid &&
        !plan_taught_response_measurement_schedule(path,request,1,{1,-1}).valid,
        "missing source clock or invalid mapped response weights cannot manufacture an anchor");
}
}
int main() {
    try {
        interior_selection();actual_feedback_owns_room();arbitrary_joint_count_and_missing_room();split_cubic_clock();response_priority_escapes_long_plateau();
        std::cout<<"Taught measurement schedule: interior per-axis room, actual feedback, split cubic 1x clock PASS\n";
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
