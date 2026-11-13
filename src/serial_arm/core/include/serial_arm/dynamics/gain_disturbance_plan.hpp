#pragma once

#include "serial_arm/dynamics/servo_gain_tuning.hpp"

namespace serial_arm::servo_tuning {

struct IndependentDisturbanceDirectionPlan {
    bool eligible{false},bidirectional{false};int direction{0};
    double required_room_rad{0},requested_torque_nm{0};
    std::string validation_scope{"independent_joint_torque_not_measured"};
    std::string reason{"insufficient_demonstrated_disturbance_room"};
};

// Room is measured from the actual experiment center to demonstrated positions
// intersected with explicit Safety. URDF expansion cannot create this evidence.
// Direction zero denotes the original symmetric input, not an absent input.
inline IndependentDisturbanceDirectionPlan independent_disturbance_direction_plan(
    double negative_room_rad,double positive_room_rad,const NoiseEstimate& noise,
    double requested_torque_nm,double kp) {
    IndependentDisturbanceDirectionPlan out;out.requested_torque_nm=requested_torque_nm;
    for(double value:{negative_room_rad,positive_room_rad,noise.encoder_step_rad,noise.rms_rad,requested_torque_nm})
        if(!std::isfinite(value) || value<0){out.reason="invalid_demonstrated_disturbance_domain";return out;}
    if(!std::isfinite(kp) || kp<=0 || requested_torque_nm<=0){out.reason="positive_restoring_gain_and_known_torque_required";return out;}
    out.required_room_rad=std::max({3*noise.encoder_step_rad,6*noise.rms_rad,2*requested_torque_nm/kp,1e-5});
    const bool negative=negative_room_rad+1e-12>=out.required_room_rad;
    const bool positive=positive_room_rad+1e-12>=out.required_room_rad;
    if(!negative && !positive)return out;
    out.eligible=true;out.bidirectional=negative && positive;
    out.direction=out.bidirectional?0:positive?1:-1;
    out.validation_scope=out.bidirectional?"bidirectional_known_joint_torque":
        positive?"positive_only_known_joint_torque":"negative_only_known_joint_torque";
    out.reason=out.bidirectional?"demonstrated_room_supports_symmetric_disturbance":
        "demonstrated_room_supports_one_sided_smooth_disturbance";
    return out;
}

// A DC-shifted multisine keeps the original three forcing frequencies. The
// same cubic edge as probe() makes the one-sided input and its slope zero at
// both ends; rectifying with abs()/squaring would alter the forcing spectrum.
inline double independent_disturbance_input(const IndependentDisturbanceDirectionPlan& plan,
    double time,const Requirements& req) {
    if(!plan.eligible)return 0;
    if(!std::isfinite(time) || !std::isfinite(req.amplitude_rad) || req.amplitude_rad<=0 ||
       !std::isfinite(req.probe_duration_s) || req.probe_duration_s<=0 ||
       !std::isfinite(plan.requested_torque_nm) || plan.requested_torque_nm<=0 ||
       req.motion_probe || std::abs(req.reference_bias_rad)>1e-12 ||
       (plan.direction!=0 && plan.direction!=1 && plan.direction!=-1))
        throw std::invalid_argument("invalid independent multisine disturbance input");
    for(double frequency:req.frequencies_hz)if(!std::isfinite(frequency) || frequency<=0)
        throw std::invalid_argument("invalid independent disturbance frequency");
    if(req.stationary_probe || time<0 || time>=req.probe_duration_s)return 0;
    const double symmetric=probe(time,req).first/req.amplitude_rad;
    if(plan.direction==0)return symmetric*plan.requested_torque_nm;
    const double edge=std::clamp(std::min(time,req.probe_duration_s-time)/.7,0.0,1.0);
    const double envelope=edge*edge*(3-2*edge);
    const double magnitude=std::clamp(.5*(envelope+symmetric),0.0,1.0)*plan.requested_torque_nm;
    return plan.direction*magnitude;
}

} // namespace serial_arm::servo_tuning
