#pragma once

#include "serial_arm/core/types.hpp"
#include <algorithm>
#include <cstdint>
#include <vector>
#include <utility>

namespace serial_arm {

struct AlignedFeedback {
    JointState state;
    std::uint64_t timestamp_ns{0};
    bool valid{false};
    bool fresh{false};
};

// Interpolate every axis to the oldest latest sample, never extrapolate.
// Each axis has independent receive times and keeps its previous measurement.
class FeedbackAligner {
public:
    void reset() { axes_.clear(); last_timestamp_ns_=0; }

    AlignedFeedback update(const JointState& state, const std::vector<std::uint64_t>& stamps,
        const std::vector<std::uint8_t>& timing_valid, std::uint64_t now_ns,
        std::uint64_t max_age_ns, std::uint64_t max_skew_ns) {
        const auto n=stamps.size();
        if(n==0 || state.pos.size()!=n || state.vel.size()!=n || state.tor.size()!=n || timing_valid.size()!=n) {
            reset(); return {};
        }
        const auto limits=std::minmax_element(stamps.begin(),stamps.end());
        if(*limits.first==0 || *limits.second>now_ns || now_ns-*limits.first>max_age_ns ||
           *limits.second-*limits.first>max_skew_ns ||
           std::any_of(timing_valid.begin(),timing_valid.end(),[](auto v){return v==0;})) {
            reset(); return {};
        }
        if(axes_.size()!=n) {reset();axes_.resize(n);}
        for(std::size_t i=0;i<n;++i) {
            auto& a=axes_[i];
            if(a.latest_ns && stamps[i]<a.latest_ns) {reset();return {};}
            if(stamps[i]>a.latest_ns) {
                a.previous_ns=a.latest_ns;
                a.previous_q=a.q;a.previous_v=a.v;a.previous_t=a.t;
                a.latest_ns=stamps[i];a.q=state.pos[i];a.v=state.vel[i];a.t=state.tor[i];
            }
        }
        const auto target=*limits.first;
        if(last_timestamp_ns_ && target<=last_timestamp_ns_)
            return AlignedFeedback{{},last_timestamp_ns_,true,false};
        JointState aligned;aligned.pos.resize(n);aligned.vel.resize(n);aligned.tor.resize(n);
        for(std::size_t i=0;i<n;++i) {
            const auto& a=axes_[i];
            if(a.latest_ns==target) {aligned.pos[i]=a.q;aligned.vel[i]=a.v;aligned.tor[i]=a.t;continue;}
            if(!a.previous_ns || a.previous_ns>target || a.latest_ns<=a.previous_ns) {
                last_timestamp_ns_=0;return {};
            }
            const double t=static_cast<double>(target-a.previous_ns)/static_cast<double>(a.latest_ns-a.previous_ns);
            aligned.pos[i]=a.previous_q+t*(a.q-a.previous_q);
            aligned.vel[i]=a.previous_v+t*(a.v-a.previous_v);
            aligned.tor[i]=a.previous_t+t*(a.t-a.previous_t);
        }
        last_timestamp_ns_=target;
        return AlignedFeedback{std::move(aligned),target,true,true};
    }
private:
    struct Axis {std::uint64_t previous_ns{0},latest_ns{0};double previous_q{0},previous_v{0},previous_t{0},q{0},v{0},t{0};};
    std::vector<Axis> axes_;
    std::uint64_t last_timestamp_ns_{0};
};
} // namespace serial_arm
