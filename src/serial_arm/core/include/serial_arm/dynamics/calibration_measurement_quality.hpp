#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <deque>
#include <vector>

namespace serial_arm::calibration_measurement {

// One cycle's actual motor-feedback observation.  No subtraction of the
// motor's PD command: at static equilibrium that torque supports gravity.
struct Observation {
    double time_s{};
    std::vector<double> q;
    std::vector<double> dq;
    std::vector<double> tau;
};

// All criteria operate on a recent window, never a single quiet velocity frame.
class StableWindow {
public:
    explicit StableWindow(double seconds=0.18): seconds_(seconds) {}
    void clear() { rows_.clear(); }
    void push(Observation row) {
        if(!valid(row)) {clear();return;}
        if(!rows_.empty() && (row.time_s <= rows_.back().time_s ||
                              row.time_s - rows_.back().time_s > 0.12 ||
                              row.q.size() != rows_.back().q.size())) clear();
        rows_.push_back(std::move(row));
        while(rows_.size()>1 && rows_.back().time_s-rows_.front().time_s > seconds_+0.02)
            rows_.pop_front();
    }
    bool stable(double max_velocity=0.03, double max_drift=0.008) const {
        if(rows_.size()<12 || rows_.back().time_s-rows_.front().time_s<seconds_*0.80) return false;
        const std::size_t n=rows_.size(), joints=rows_.front().q.size();
        for(std::size_t j=0;j<joints;++j) {
            double minq=rows_[0].q[j], maxq=minq, sum=0.0, sumsq=0.0;
            double first=0.0, second=0.0;
            for(std::size_t k=0;k<n;++k) {
                const auto& row=rows_[k];
                if(std::abs(row.dq[j])>max_velocity) return false;
                minq=std::min(minq,row.q[j]); maxq=std::max(maxq,row.q[j]);
                sum+=row.tau[j]; sumsq+=row.tau[j]*row.tau[j];
                if(k<n/2) first+=row.tau[j]; else second+=row.tau[j];
            }
            if(maxq-minq>max_drift) return false;
            const double mean=sum/static_cast<double>(n);
            const double stddev=std::sqrt(std::max(0.0,sumsq/n-mean*mean));
            const double max_std=std::max(0.10,0.035*std::abs(mean));
            const double half_diff=std::abs(first/(n/2)-second/(n-n/2));
            if(stddev>max_std || half_diff>std::max(0.12,0.05*std::abs(mean))) return false;
        }
        return true;
    }
    std::size_t size() const {return rows_.size();}
private:
    static bool valid(const Observation& r) {
        if(r.q.empty() || r.q.size()!=r.dq.size() || r.q.size()!=r.tau.size() || !std::isfinite(r.time_s)) return false;
        for(std::size_t i=0;i<r.q.size();++i)
            if(!std::isfinite(r.q[i]) || !std::isfinite(r.dq[i]) || !std::isfinite(r.tau[i])) return false;
        return true;
    }
    double seconds_;
    std::deque<Observation> rows_;
};

} // namespace serial_arm::calibration_measurement
