#include "serial_arm/core/reference_governor.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace serial_arm {
namespace {
bool finite(const JointVector& value, std::size_t n) {
    return value.size()==n && std::all_of(value.begin(),value.end(),[](double x){return std::isfinite(x);});
}
double dot(const JointVector& a, const JointVector& b) {
    return std::inner_product(a.begin(),a.end(),b.begin(),0.0);
}
struct HalfSpace { JointVector normal; double bound; double multiplier{0.0}; };
}

tl::expected<ReferenceGovernorOutput, ReferenceGovernorErr>
govern_reference(const ReferenceGovernorInput& input) {
    const auto n=input.requested.pos.size();
    const auto& limits=input.limits;
    if(n==0 || !std::isfinite(input.dt) || input.dt<=0.0 ||
       !finite(input.requested.pos,n) || !finite(input.requested.vel,n) ||
       !finite(input.requested.tor,n) || !finite(input.requested.kp,n) || !finite(input.requested.kd,n) ||
       !finite(input.measured.pos,n) || !finite(input.measured.vel,n) ||
       !finite(input.previous.pos,n) || !finite(input.previous.vel,n) || !finite(input.requested_acceleration,n) ||
       limits.has_position_limit.size()!=n || !finite(limits.min_pos,n) || !finite(limits.max_pos,n) ||
       !finite(limits.pos_margin,n) || !finite(limits.max_vel,n) || !finite(limits.max_acc,n) ||
       !finite(limits.max_effort,n) || !finite(limits.max_kp,n) || !finite(limits.max_kd,n))
        return tl::make_unexpected(ReferenceGovernorErr::INVALID_INPUT);
    if(input.model_admittance && (input.mass_matrix.size()!=n ||
       !std::all_of(input.mass_matrix.begin(),input.mass_matrix.end(),[&](const auto& row){return finite(row,n);})))
        return tl::make_unexpected(ReferenceGovernorErr::INVALID_INPUT);

    const double h=input.dt;
    JointVector velocity(n), lower(n), upper(n), requested_total(n), ff_constant(n), total_constant(n);
    std::vector<JointVector> ff_matrix(n,JointVector(n,0.0)), total_matrix(n,JointVector(n,0.0));
    std::vector<HalfSpace> constraints;
    constraints.reserve(6*n);
    bool impossible=false;
    const auto append=[&](JointVector normal,double bound) {
        const double norm=std::sqrt(dot(normal,normal));
        if(!std::isfinite(norm)||!std::isfinite(bound)) { impossible=true;return; }
        if(norm<1e-14) { if(bound < -1e-10)impossible=true;return; }
        for(double& x:normal)x/=norm;
        constraints.push_back({std::move(normal),bound/norm,0.0});
    };
    for(std::size_t i=0;i<n;++i) {
        if(limits.max_vel[i]<=0 || limits.max_acc[i]<=0 || limits.max_effort[i]<=0 ||
           input.requested.kp[i]<0 || input.requested.kd[i]<0 ||
           input.requested.kp[i]>limits.max_kp[i] || input.requested.kd[i]>limits.max_kd[i])
            return tl::make_unexpected(ReferenceGovernorErr::INVALID_INPUT);
        lower[i]=std::max(-limits.max_vel[i],input.previous.vel[i]-limits.max_acc[i]*h);
        upper[i]=std::min(limits.max_vel[i],input.previous.vel[i]+limits.max_acc[i]*h);
        if(limits.has_position_limit[i]) {
            lower[i]=std::max(lower[i],(limits.min_pos[i]+limits.pos_margin[i]-input.previous.pos[i])/h);
            upper[i]=std::min(upper[i],(limits.max_pos[i]-limits.pos_margin[i]-input.previous.pos[i])/h);
        }
        if(lower[i]>upper[i])return tl::make_unexpected(ReferenceGovernorErr::INFEASIBLE);
        velocity[i]=(input.requested.pos[i]-input.previous.pos[i])/h;
        JointVector axis(n,0.0);axis[i]=1.0;append(axis,upper[i]);axis[i]=-1.0;append(axis,-lower[i]);
        ff_constant[i]=input.requested.tor[i];
        if(input.model_admittance) {
            for(std::size_t j=0;j<n;++j) {
                ff_matrix[i][j]=input.mass_matrix[i][j]/h;
                ff_constant[i]-=input.mass_matrix[i][j]*(input.previous.vel[j]/h+input.requested_acceleration[j]);
            }
        }
        total_matrix[i]=ff_matrix[i];
        total_matrix[i][i]+=input.requested.kp[i]*h+input.requested.kd[i];
        total_constant[i]=ff_constant[i]+input.requested.kp[i]*(input.previous.pos[i]-input.measured.pos[i])
            -input.requested.kd[i]*input.measured.vel[i];
        requested_total[i]=input.requested.tor[i]+input.requested.kp[i]*(input.requested.pos[i]-input.measured.pos[i])
            +input.requested.kd[i]*(input.requested.vel[i]-input.measured.vel[i]);
        for(const auto& pair : {std::make_pair(ff_matrix[i],ff_constant[i]),std::make_pair(total_matrix[i],total_constant[i])}) {
            append(pair.first,limits.max_effort[i]-pair.second);
            JointVector opposite=pair.first;for(double& x:opposite)x=-x;
            append(std::move(opposite),limits.max_effort[i]+pair.second);
        }
    }
    if(impossible)return tl::make_unexpected(ReferenceGovernorErr::INFEASIBLE);

    // Hildreth's dual coordinate projection solves the small strictly convex
    // velocity QP. Full mass-matrix rows retain cross-joint torque coupling
    bool converged=false;
    for(int iteration=0;iteration<1024;++iteration) {
        double largest_change=0.0;
        for(auto& constraint:constraints) {
            const double next=std::max(0.0,constraint.multiplier+dot(constraint.normal,velocity)-constraint.bound);
            const double change=next-constraint.multiplier;
            constraint.multiplier=next;
            largest_change=std::max(largest_change,std::abs(change));
            for(std::size_t j=0;j<n;++j)velocity[j]-=change*constraint.normal[j];
        }
        double violation=0.0;
        for(const auto& constraint:constraints)violation=std::max(violation,dot(constraint.normal,velocity)-constraint.bound);
        if(violation<1e-13 && largest_change<1e-12) { converged=true;break; }
    }
    if(!converged)return tl::make_unexpected(ReferenceGovernorErr::INFEASIBLE);
    ReferenceGovernorOutput output;
    output.accepted=input.requested;
    output.acceleration.resize(n);output.predicted_total_torque.resize(n);
    output.reference_limited.assign(n,0);output.torque_limited.assign(n,0);
    for(std::size_t i=0;i<n;++i) {
        output.accepted.vel[i]=velocity[i];
        output.accepted.pos[i]=input.previous.pos[i]+h*velocity[i];
        output.acceleration[i]=(velocity[i]-input.previous.vel[i])/h;
        output.accepted.tor[i]=ff_constant[i]+dot(ff_matrix[i],velocity);
        output.predicted_total_torque[i]=total_constant[i]+dot(total_matrix[i],velocity);
        const double tolerance=1e-8*std::max(1.0,limits.max_effort[i]);
        if(!std::isfinite(output.predicted_total_torque[i]) ||
           std::abs(output.predicted_total_torque[i])>limits.max_effort[i]+tolerance ||
           std::abs(output.accepted.tor[i])>limits.max_effort[i]+tolerance)
            return tl::make_unexpected(ReferenceGovernorErr::INFEASIBLE);
        output.reference_limited[i]=std::abs(output.accepted.pos[i]-input.requested.pos[i])>1e-9 ||
            std::abs(output.accepted.vel[i]-input.requested.vel[i])>1e-9;
        output.torque_limited[i]=std::abs(requested_total[i])>limits.max_effort[i]+tolerance ||
            std::abs(input.requested.tor[i])>limits.max_effort[i]+tolerance;
    }
    return output;
}
} // namespace serial_arm
