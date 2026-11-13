#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <stdexcept>
#include <utility>
#include <vector>

namespace serial_arm::calibration_retime {

// Geometric retiming for model calibration only. Unlike replaying the teaching
// clock, this preserves joint-space waypoints while discarding pauses and
// choosing a NEW time axis from the configured command velocity/acceleration
// limits. It never expands those limits to hit a requested time budget
struct Plan {
    std::vector<std::vector<double>> positions;
    double sample_dt{0.0};
    double single_pass_s{0.0};
    std::size_t original_samples{0};
    std::size_t geometric_waypoints{0};
    double joint_path_length_rad{0.0};
};

// Uniform-knot, shape-preserving Hermite slope. A stationary plateau or local
// extremum has zero slope; same-direction adjacent secants use their harmonic
// mean. This keeps each cubic inside its two measured endpoint values while
// retaining a continuous, real derivative rather than clipping positions.
inline double node_tangent(const std::vector<std::vector<double>>& positions,std::size_t knot,std::size_t joint) {
    if(knot==0 || knot+1==positions.size())return 0.0;
    const double left=positions[knot][joint]-positions[knot-1][joint];
    const double right=positions[knot+1][joint]-positions[knot][joint];
    if(left==0 || right==0 || (left>0)!=(right>0))return 0.0;
    return std::abs(left)<=std::abs(right)?left/(.5+.5*left/right):right/(.5+.5*right/left);
}

// Locate ordered stops on their first local approach, not the closest pose
// anywhere in the remaining path. An out-and-back demonstration can revisit
// exactly the same pose later with a smaller floating-point residual; choosing
// that later visit consumes subsequent stops and reverses their intended order.
inline std::vector<std::size_t> ordered_stop_indices(
    const std::vector<std::vector<double>>& positions,
    const std::vector<std::vector<double>>& stops,
    double maximum_distance_rad,
    bool interior_only = true) {
    if(stops.empty()) return {};
    if(positions.size() < (interior_only ? 3U : 1U) || positions.front().empty() ||
       !std::isfinite(maximum_distance_rad) || maximum_distance_rad < 0.0)
        throw std::invalid_argument("invalid calibration stop mapping");
    const auto joints = positions.front().size();
    for(const auto& q : positions) {
        if(q.size() != joints) throw std::invalid_argument("invalid calibration stop path dimension");
        for(double value : q) if(!std::isfinite(value))
            throw std::invalid_argument("non-finite calibration stop path");
    }
    const std::size_t end = positions.size() - (interior_only ? 1U : 0U);
    std::size_t cursor = interior_only ? 1U : 0U;
    std::vector<std::size_t> result;
    result.reserve(stops.size());
    for(const auto& stop : stops) {
        if(stop.size() != joints) throw std::invalid_argument("invalid calibration stop dimension");
        for(double value : stop) if(!std::isfinite(value))
            throw std::invalid_argument("non-finite calibration stop");
        std::size_t best = end;
        double best_distance = std::numeric_limits<double>::infinity();
        for(std::size_t k = cursor; k < end; ++k) {
            double distance = 0.0;
            for(std::size_t j = 0; j < joints; ++j)
                distance = std::max(distance, std::abs(stop[j] - positions[k][j]));
            if(best == end) {
                if(distance <= maximum_distance_rad) { best = k; best_distance = distance; }
            } else if(distance < best_distance) {
                best = k; best_distance = distance;
            } else if(distance > best_distance + 1e-12) {
                // Numerical plateau noise may still belong to this visit;
                // a genuine departure ends it before any later revisit.
                break;
            }
        }
        if(best == end) throw std::runtime_error("unable to preserve calibration stop on retimed path");
        result.push_back(best);
        // Retiming may merge distinct nearby source poses into one geometric
        // knot. Ordered targets may share that knot when each independently
        // satisfies the original tolerance; requiring a fresh knot for every
        // target would push the last nearby stop onto a later return visit.
        cursor = best;
    }
    return result;
}

inline Plan plan(const std::vector<std::vector<double>>& input,
                 const std::vector<double>& max_vel,
                 const std::vector<double>& max_acc,
                 double ctrl_frequency_hz,
                 double tolerance_rad = 0.003,
                 const std::vector<std::vector<double>>& static_stops = {}) {
    const std::size_t joints = max_vel.size();
    if(input.size() < 20 || joints == 0 || max_acc.size() != joints ||
       !std::isfinite(ctrl_frequency_hz) || ctrl_frequency_hz <= 0.0)
        throw std::invalid_argument("invalid calibration demonstration or joint limits");
    for(std::size_t j = 0; j < joints; ++j) {
        if(!std::isfinite(max_vel[j]) || max_vel[j] <= 0.0 ||
           !std::isfinite(max_acc[j]) || max_acc[j] <= 0.0)
            throw std::invalid_argument("calibration requires positive joint speed and acceleration limits");
    }
    for(const auto& point : input) {
        if(point.size() != joints) throw std::invalid_argument("inconsistent demonstration joint dimensions");
        for(double q : point) if(!std::isfinite(q)) throw std::invalid_argument("non-finite demonstrated joint position");
    }

    // Ramer-Douglas-Peucker in joint-space: use spatial distance to a segment,
    // not index/time distance (which would preserve long periods of standing still)
    std::vector<bool> keep(input.size(), false);
    keep.front() = keep.back() = true;
    std::vector<std::pair<std::size_t,std::size_t>> intervals{{0,input.size()-1}};
    while(!intervals.empty()) {
        const auto [a,b] = intervals.back(); intervals.pop_back();
        if(b - a < 2) continue;
        double denominator = 0.0;
        for(std::size_t j=0;j<joints;++j) denominator += (input[b][j]-input[a][j])*(input[b][j]-input[a][j]);
        double worst = -1.0;
        std::size_t worst_index = a;
        for(std::size_t k=a+1;k<b;++k) {
            double numerator = 0.0;
            for(std::size_t j=0;j<joints;++j) numerator += (input[k][j]-input[a][j])*(input[b][j]-input[a][j]);
            const double t = denominator > 1e-15 ? std::clamp(numerator/denominator, 0.0, 1.0) : 0.0;
            double deviation = 0.0;
            for(std::size_t j=0;j<joints;++j) deviation = std::max(deviation,std::abs(input[k][j] - (input[a][j]+t*(input[b][j]-input[a][j]))));
            if(deviation > worst) { worst = deviation; worst_index = k; }
        }
        if(worst > tolerance_rad) {
            keep[worst_index]=true;
            intervals.emplace_back(a,worst_index);
            intervals.emplace_back(worst_index,b);
        }
    }
    std::vector<std::vector<double>> corners;
    for(std::size_t k=0;k<input.size();++k) if(keep[k]) corners.push_back(input[k]);
    std::vector<double> arclength(corners.size(),0.0);
    for(std::size_t k=1;k<corners.size();++k) {
        double sum=0.0;
        for(std::size_t j=0;j<joints;++j) sum+=(corners[k][j]-corners[k-1][j])*(corners[k][j]-corners[k-1][j]);
        arclength[k]=arclength[k-1]+std::sqrt(sum);
    }
    const double length=arclength.back();
    if(length < 0.04) throw std::invalid_argument("demonstration does not contain enough joint movement for calibration");
    // Sample the demonstrated polyline in joint-space arclength. Retiming
    // this *continuous path* avoids a separate accelerate/decelerate cycle
    // for each recorded encoder tick or geometry waypoint
    const std::size_t count = std::clamp<std::size_t>(static_cast<std::size_t>(std::ceil(length / 0.04)) + 1, 120, 1600);
    const double ds = length / static_cast<double>(count - 1);
    Plan result;
    result.original_samples = input.size();
    result.geometric_waypoints = corners.size();
    result.joint_path_length_rad = length;
    std::vector<std::vector<double>> geometry(count, std::vector<double>(joints, 0.0));
    std::size_t segment = 1;
    for(std::size_t k=0;k<count;++k) {
        const double distance = static_cast<double>(k)*ds;
        while(segment+1<corners.size() && arclength[segment]<distance) ++segment;
        const double width=arclength[segment]-arclength[segment-1];
        const double alpha=width>1e-12 ? std::clamp((distance-arclength[segment-1])/width,0.0,1.0) : 0.0;
        for(std::size_t j=0;j<joints;++j)
            geometry[k][j] = corners[segment-1][j] + alpha*(corners[segment][j]-corners[segment-1][j]);
    }
    // Small bounded geometric corner rounding removes quantization-induced
    // velocity discontinuities. Reject if smoothing strays too far from the
    // manually demonstrated geometry (per joint 0.01 rad max)
    const auto unsmoothed = geometry;
    for(std::size_t k=1;k+1<count;++k) for(std::size_t j=0;j<joints;++j) {
        geometry[k][j] = unsmoothed[k-1][j]==unsmoothed[k][j] && unsmoothed[k][j]==unsmoothed[k+1][j] ? unsmoothed[k][j] :
            0.12*unsmoothed[k-1][j] + 0.76*unsmoothed[k][j] + 0.12*unsmoothed[k+1][j];
        if(std::abs(geometry[k][j]-unsmoothed[k][j])>0.01)
            throw std::runtime_error("demonstrated path has a corner too sharp to safely smooth");
    }
    std::vector<std::vector<double>> tangent(count,std::vector<double>(joints,0.0));
    std::vector<std::vector<double>> curvature(count,std::vector<double>(joints,0.0));
    for(std::size_t k=0;k<count;++k) for(std::size_t j=0;j<joints;++j){
        tangent[k][j] = node_tangent(geometry,k,j)/ds;
    }
    for(std::size_t k=1;k+1<count;++k) for(std::size_t j=0;j<joints;++j)
        curvature[k][j] = (tangent[k+1][j]-tangent[k-1][j])/(2.0*ds);
    // Allow higher speed on smooth segments while retaining explicit joint
    // limits, curve-aware braking and the final spline-extrema verification
    std::vector<double> speed(count,std::numeric_limits<double>::infinity());
    std::vector<double> accel(count,std::numeric_limits<double>::infinity());
    for(std::size_t k=0;k<count;++k) for(std::size_t j=0;j<joints;++j) {
        const double slope=std::abs(tangent[k][j]);
        const double bend=std::abs(curvature[k][j]);
        speed[k]=std::min(speed[k],0.94*max_vel[j]/std::max(slope,1.0e-8));
        speed[k]=std::min(speed[k],std::sqrt(0.80*max_acc[j]/std::max(bend,1.0e-8)));
        accel[k]=std::min(accel[k],0.80*max_acc[j]/std::max(slope,1.0e-8));
    }
    // Apply bounds for the actual cubic geometry between each pair of knots:
    // finite-difference curvature at knots alone misses sharp intermediate
    // curvature, which otherwise triggers global slowdown after resampling
    for(std::size_t k=0;k+1<count;++k) for(std::size_t j=0;j<joints;++j) {
        const double d=geometry[k+1][j]-geometry[k][j];
        const double m0=tangent[k][j]*ds, m1=tangent[k+1][j]*ds;
        const double max_d1=std::max({std::abs(m0), std::abs(3*d-m0-m1),std::abs(m1)})/ds;
        const double max_d2=std::max(std::abs(6*d-4*m0-2*m1),std::abs(-6*d+2*m0+4*m1))/(ds*ds);
        const double v_speed=0.94*max_vel[j]/std::max(max_d1,1e-8);
        const double v_accel=std::sqrt(0.80*max_acc[j]/std::max(max_d2,1e-8));
        speed[k]=std::min({speed[k],v_speed,v_accel});
        speed[k+1]=std::min({speed[k+1],v_speed,v_accel});
        accel[k]=std::min(accel[k],0.80*max_acc[j]/std::max(max_d1,1e-8));
        accel[k+1]=std::min(accel[k+1],0.80*max_acc[j]/std::max(max_d1,1e-8));
    }
    // Preserve the geometric speed allowance before forcing collection stops.
    // Two nearby legal stops may occupy adjacent knots; their intervening
    // finite distance needs an accelerate/brake profile, not a zero divisor.
    const auto geometric_speed_limit = speed;
    // Plan static collection points as genuine zero-speed boundary conditions.
    // Zero speed at a curve knot is handled by the same forward/backward braking
    // passes as path endpoints; the reverse traversal inherits these stops
    for(const auto best_index : ordered_stop_indices(geometry, static_stops, 0.03, false)) {
        // The beginning and end are already zero-speed boundary conditions
        if(best_index > 0 && best_index + 1 < count) speed[best_index] = 0.0;
    }
    speed.front()=0.0;
    speed.back()=0.0;
    for(std::size_t k=1;k<count;++k)
        speed[k]=std::min(speed[k],std::sqrt(speed[k-1]*speed[k-1]+2.0*ds*std::min(accel[k-1],accel[k])));
    for(std::size_t k=count-1;k>0;--k)
        speed[k-1]=std::min(speed[k-1],std::sqrt(speed[k]*speed[k]+2.0*ds*std::min(accel[k-1],accel[k])));
    std::vector<double> time(count,0.0),rest_peak_speed(count,0.0),rest_acceleration(count,0.0);
    for(std::size_t k=1;k<count;++k) {
        const double denominator = speed[k-1]+speed[k];
        if(denominator>0.0) {
            time[k]=time[k-1]+2.0*ds/denominator;
        } else {
            const double acceleration=std::min(accel[k-1],accel[k]);
            const double peak=std::min({std::sqrt(acceleration*ds),geometric_speed_limit[k-1],geometric_speed_limit[k]});
            if(!std::isfinite(acceleration) || acceleration<=0 || !std::isfinite(peak) || peak<=0)
                throw std::runtime_error("cannot calculate continuous calibration retiming");
            rest_peak_speed[k]=peak;rest_acceleration[k]=acceleration;
            const double ramp=peak/acceleration;
            const double cruise=std::max(0.0,ds-peak*ramp)/peak;
            time[k]=time[k-1]+2*ramp+cruise;
        }
    }
    const double total=time.back();
    if(!std::isfinite(total) || total<=0.0 || total>36000.0)
        throw std::runtime_error("calibration trajectory duration is invalid or excessive");
    // Cubic Hermite interpolation is performed by the production replay interface at the same
    // uniform sampling interval; check its exact derivative extrema below
    const std::size_t samples=std::max<std::size_t>(2,static_cast<std::size_t>(std::ceil(total/0.01))+1);
    if(samples>200000) throw std::runtime_error("calibration trajectory exceeds safe sample count");
    result.sample_dt=total/static_cast<double>(samples-1);
    result.positions.reserve(samples);
    std::size_t seg=0;
    for(std::size_t k=0;k<samples;++k) {
        const double t=k==samples-1?total:static_cast<double>(k)*result.sample_dt;
        while(seg+1<count-1 && time[seg+1]<t) ++seg;
        const double tau=std::clamp(t-time[seg],0.0,time[seg+1]-time[seg]);
        const double dt=time[seg+1]-time[seg];
        const double accel_s=(speed[seg+1]-speed[seg])/dt;
        double distance=speed[seg]*tau + 0.5*accel_s*tau*tau;
        if(rest_peak_speed[seg+1]>0) {
            const double peak=rest_peak_speed[seg+1],acceleration=rest_acceleration[seg+1];
            const double ramp=peak/acceleration;
            if(tau<ramp)distance=.5*acceleration*tau*tau;
            else if(tau<=dt-ramp)distance=.5*peak*ramp+peak*(tau-ramp);
            else distance=ds-.5*acceleration*(dt-tau)*(dt-tau);
        }
        const double along=std::clamp(distance/ds,0.0,1.0);
        std::vector<double> q(joints,0.0);
        for(std::size_t j=0;j<joints;++j) {
            if(geometry[seg][j]==geometry[seg+1][j]){q[j]=geometry[seg][j];continue;}
            const double u=along, u2=u*u, u3=u2*u;
            const double m0=tangent[seg][j]*ds, m1=tangent[seg+1][j]*ds;
            q[j]=(2*u3-3*u2+1)*geometry[seg][j]+(u3-2*u2+u)*m0
                +(-2*u3+3*u2)*geometry[seg+1][j]+(u3-u2)*m1;
        }
        result.positions.push_back(std::move(q));
    }
    // Validate the *actual* Hermite controller interpolation. The local path
    // envelope is the primary limiter; this final factor is only a conservative
    // correction for reconstruction between time samples
    double slowdown=1.0;
    for(std::size_t k=1;k<samples;++k) for(std::size_t j=0;j<joints;++j) {
        const double p0=result.positions[k-1][j], p1=result.positions[k][j];
        const double m0=node_tangent(result.positions,k-1,j);
        const double m1=node_tangent(result.positions,k,j);
        const double d=p1-p0;
        const double d1=std::max({std::abs(m0),std::abs(3*d-m0-m1),std::abs(m1)});
        const double d2=std::max(std::abs(6*d-4*m0-2*m1),std::abs(-6*d+2*m0+4*m1));
        slowdown=std::max({slowdown,d1/(0.94*max_vel[j]*result.sample_dt),
            std::sqrt(d2/(0.90*max_acc[j]*result.sample_dt*result.sample_dt))});
    }
    result.sample_dt*=slowdown;
    result.single_pass_s=result.sample_dt*static_cast<double>(samples-1);
    if(!std::isfinite(result.single_pass_s)) throw std::runtime_error("invalid retimed calibration duration");
    return result;
}

struct Sample {
    std::vector<double> position;
    std::vector<double> velocity;
};

inline Sample interpolate(const std::vector<std::vector<double>>& positions,
                          double progress, double sample_dt, double playback_rate) {
    if(positions.size()<2 || !std::isfinite(sample_dt) || !(sample_dt>0.0) ||
       !std::isfinite(progress) || !std::isfinite(playback_rate) || playback_rate<0.0)
        throw std::invalid_argument("invalid calibration playback state");
    const auto n=positions.size();
    const double clipped=std::clamp(progress,0.0,static_cast<double>(n-1));
    const std::size_t lower=std::min(static_cast<std::size_t>(std::floor(clipped)),n-2);
    const double u=clipped-static_cast<double>(lower);
    Sample result;
    result.position.resize(positions[lower].size());
    result.velocity.resize(positions[lower].size());
    for(std::size_t j=0;j<result.position.size();++j){
        const double p0=positions[lower][j], p1=positions[lower+1][j];
        if(p0==p1){result.position[j]=p0;result.velocity[j]=0.0;continue;}
        const double m0=node_tangent(positions,lower,j);
        const double m1=node_tangent(positions,lower+1,j);
        const double u2=u*u, u3=u2*u;
        result.position[j]=(2*u3-3*u2+1)*p0+(u3-2*u2+u)*m0+(-2*u3+3*u2)*p1+(u3-u2)*m1;
        result.velocity[j]=((6*u2-6*u)*p0+(3*u2-4*u+1)*m0+(-6*u2+6*u)*p1+(3*u2-2*u)*m1)*playback_rate/sample_dt;
    }
    return result;
}

} // namespace serial_arm::calibration_retime
