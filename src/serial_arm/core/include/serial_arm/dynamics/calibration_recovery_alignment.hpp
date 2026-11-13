#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

namespace serial_arm::calibration_recovery {
constexpr double kPositionToleranceRad = 0.08;
constexpr double kVelocityToleranceRadS = 0.05;
constexpr double kFeedbackMaxAgeMs = 500.0;
inline double position_tolerance(const std::string& type) { return type=="prismatic" ? .001 : kPositionToleranceRad; }
inline double velocity_tolerance(const std::string& type) { return type=="prismatic" ? .001 : kVelocityToleranceRadS; }
inline double resume_tolerance(const std::string& type) { return type=="prismatic" ? .001 : .05; }
struct Choice {
    bool valid{false},reverse_recorded_path{false},at_endpoint{false};
    double max_position_ratio{std::numeric_limits<double>::infinity()};
    double max_velocity_ratio{std::numeric_limits<double>::infinity()};
};
struct AutomaticEndpointContinuation {
    bool valid{false};
    std::vector<double> reference_origin;
};
inline bool emitted_endpoint_matches(const std::vector<double>& accepted_reference,
    const std::vector<double>& target) {
    if(target.empty() || accepted_reference.size()!=target.size())return false;
    for(std::size_t i=0;i<target.size();++i)if(!std::isfinite(accepted_reference[i]) ||
        !std::isfinite(target[i]) || std::abs(accepted_reference[i]-target[i])>1e-8)return false;
    return true;
}
// A completed, actually emitted endpoint is a continuation credential, not an
// accuracy result. The next reference begins at the current accepted command;
// delayed/weak actual tracking cannot turn an automatic traversal into another
// manual alignment step. A cold task or a different endpoint has no credential.
inline AutomaticEndpointContinuation automatic_endpoint_continuation(
    const std::vector<double>& actual,const std::vector<double>& accepted_reference,
    const std::vector<double>& completed_endpoint,const std::vector<double>& target,
    const std::vector<double>& feedback_age_ms,bool active) {
    AutomaticEndpointContinuation out;const auto n=target.size();
    if(!active || !n || actual.size()!=n || accepted_reference.size()!=n ||
       completed_endpoint.size()!=n || feedback_age_ms.size()!=n)return out;
    for(std::size_t i=0;i<n;++i) {
        if(!std::isfinite(actual[i]) || !std::isfinite(accepted_reference[i]) ||
           !std::isfinite(completed_endpoint[i]) || !std::isfinite(target[i]) ||
           !std::isfinite(feedback_age_ms[i]) || feedback_age_ms[i]<0 || feedback_age_ms[i]>60 ||
           std::abs(completed_endpoint[i]-target[i])>1e-8)return out;
    }
    out.valid=true;out.reference_origin=accepted_reference;return out;
}
inline std::vector<double> endpoint_bridge_reference(const std::vector<double>& origin,
    const std::vector<double>& target,double progress) {
    if(origin.empty() || origin.size()!=target.size() || !std::isfinite(progress))return {};
    const double u=std::clamp(progress,0.0,1.0),w=u*u*(3-2*u);
    std::vector<double> out(origin.size());
    for(std::size_t i=0;i<out.size();++i) {
        if(!std::isfinite(origin[i]) || !std::isfinite(target[i]))return {};
        out[i]=(1-w)*origin[i]+w*target[i];
    }
    return out;
}
inline Choice select_endpoint(const std::vector<double>& actual,
                             const std::vector<double>& velocity,
                             const std::vector<double>& recorded_start,
                             const std::vector<double>& recorded_end,
                             double feedback_age_ms,
                             const std::vector<std::string>& joint_types={}) {
    Choice result;
    if(actual.empty() || actual.size()!=velocity.size() || actual.size()!=recorded_start.size() || actual.size()!=recorded_end.size() ||
       (!joint_types.empty() && joint_types.size()!=actual.size()) ||
       !std::isfinite(feedback_age_ms) || feedback_age_ms<0 || feedback_age_ms>kFeedbackMaxAgeMs)return result;
    double first=0,last=0,vmax=0;
    for(std::size_t i=0;i<actual.size();++i) {
        if(!std::isfinite(actual[i]) || !std::isfinite(velocity[i]) || !std::isfinite(recorded_start[i]) || !std::isfinite(recorded_end[i]))return result;
        const auto type=joint_types.empty()?"revolute":joint_types[i];
        first=std::max(first,std::abs(actual[i]-recorded_start[i])/position_tolerance(type));
        last=std::max(last,std::abs(actual[i]-recorded_end[i])/position_tolerance(type));
        vmax=std::max(vmax,std::abs(velocity[i])/velocity_tolerance(type));
    }
    result.valid=true;result.reverse_recorded_path=first<last;
    result.max_position_ratio=std::min(first,last);result.max_velocity_ratio=vmax;
    result.at_endpoint=result.max_position_ratio<=1 && vmax<=1;
    return result;
}
}
