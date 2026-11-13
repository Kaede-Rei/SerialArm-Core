#include "serial_arm/dynamics/gain_taught_anchor_visit.hpp"
#include "serial_arm/dynamics/gain_taught_measurement_schedule.hpp"

#include <iostream>
#include <numeric>
#include <stdexcept>

using namespace serial_arm::servo_tuning;
namespace {
void check(bool condition,const char* message) {if(!condition)throw std::runtime_error(message);}
void close(double a,double b,const char* message,double tolerance=1e-9) {
    check(std::abs(a-b)<=tolerance,message);
}

void validate_visit(const TaughtAnchorVisitPlan& plan,const std::vector<double>& vmax,
                    const std::vector<double>& amax) {
    check(plan.valid && plan.duration_s>0,"visit did not produce executable timing");
    const auto start=sample_taught_anchor_visit(plan,0);
    const auto end=sample_taught_anchor_visit(plan,plan.duration_s);
    check(start.valid && !start.complete && end.complete,"visit completion state wrong");
    close(start.source_progress,plan.from_source_index,"visit starts on wrong branch");
    close(end.source_progress,plan.to_source_index,"visit finishes on wrong branch");
    for(std::size_t j=0;j<vmax.size();++j) {
        close(start.position[j],plan.positions[plan.from_source_index][j],"start position differs from original path");
        close(end.position[j],plan.positions[plan.to_source_index][j],"end position differs from original path");
        close(start.velocity[j],0,"start velocity is not zero");close(end.velocity[j],0,"end velocity is not zero");
        close(start.acceleration[j],0,"start acceleration is not zero");close(end.acceleration[j],0,"end acceleration is not zero");
    }
    const std::size_t samples=std::max<std::size_t>(2000,std::ceil(plan.duration_s/.002));
    double previous=plan.from_source_index;
    bool moved=false;
    for(std::size_t k=0;k<=samples;++k) {
        const double t=plan.duration_s*k/samples;
        const auto s=sample_taught_anchor_visit(plan,t);
        check(s.valid,"sample unexpectedly invalid");
        check(plan.direction*(s.source_progress-previous)>=-1e-10,"source path reversed or jumped branches");
        check(s.source_progress>=std::min(plan.from_source_index,plan.to_source_index)-1e-10 &&
              s.source_progress<=std::max(plan.from_source_index,plan.to_source_index)+1e-10,
              "source time left authorized path interval");
        if(k)check(std::abs(s.source_progress-previous)<=plan.duration_s/samples/plan.sample_dt+1e-9,
                   "visit skipped source path rather than traversing at at most 1x");
        moved=moved || std::abs(s.source_progress-plan.from_source_index)>.1;
        previous=s.source_progress;
        // Production replay's independent interpolator defines the exact
        // existing path; interpolation of just visit endpoints is not allowed.
        const auto original=serial_arm::calibration_retime::interpolate(plan.positions,s.source_progress,plan.sample_dt,1);
        const auto lower=std::min(s.source_index,plan.positions.size()-2);
        for(std::size_t j=0;j<vmax.size();++j) {
            close(s.position[j],original.position[j],"visit took a geometric shortcut");
            close(s.velocity[j],original.velocity[j]*s.source_rate,"reported reference velocity is inconsistent with time map");
            check(s.position[j]>=std::min(plan.positions[lower][j],plan.positions[lower+1][j])-1e-9 &&
                  s.position[j]<=std::max(plan.positions[lower][j],plan.positions[lower+1][j])+1e-9,
                  "visit left original segment range");
            check(std::abs(s.velocity[j])<=vmax[j]+1e-8,"actual reference exceeds supplied velocity limit");
            check(std::abs(s.acceleration[j])<=amax[j]+1e-8,"actual reference exceeds supplied acceleration limit");
            if(k%53==0 && t>1e-4 && t+1e-4<plan.duration_s &&
               std::abs(s.source_progress-std::round(s.source_progress))>1e-3) {
                const auto before=sample_taught_anchor_visit(plan,t-1e-5);
                const auto after=sample_taught_anchor_visit(plan,t+1e-5);
                close((after.position[j]-before.position[j])/2e-5,s.velocity[j],"reference velocity is not position derivative",2e-6);
                close((after.velocity[j]-before.velocity[j])/2e-5,s.acceleration[j],"reference acceleration is not velocity derivative",2e-6);
            }
        }
    }
    check(moved,"nonzero source interval was silently omitted");
    // The original shape-preserving Hermite curve is C1 at its own knots.
    // The added time map must be C2, with C2 rest entry and exit in joint space.
    for(double join:{0.0,plan.ramp_s,plan.ramp_s+plan.cruise_s,plan.duration_s}) {
        const auto left=sample_taught_anchor_visit(plan,join-1e-7);
        const auto right=sample_taught_anchor_visit(plan,join+1e-7);
        close(left.source_time_s,right.source_time_s,"source-time join position discontinuity",3e-7);
        close(left.source_rate,right.source_rate,"source-time join velocity discontinuity",3e-7);
        close(left.source_acceleration,right.source_acceleration,"source-time join acceleration discontinuity",3e-7);
        if(join==0 || join==plan.duration_s)for(std::size_t j=0;j<vmax.size();++j) {
            close(left.position[j],right.position[j],"rest join q discontinuity",1e-8);
            close(left.velocity[j],right.velocity[j],"rest join dq discontinuity",1e-8);
            close(left.acceleration[j],right.acceleration[j],"rest join ddq discontinuity",1e-8);
        }
    }
    const auto after=sample_taught_anchor_visit(plan,plan.duration_s+3);
    check(after.complete && after.position==end.position && after.velocity==end.velocity,
          "completion does not remain at exact original anchor");
}

void arbitrary_axes_and_permutation() {
    for(std::size_t joints:{1U,3U,7U}) {
        std::vector<std::vector<double>> path(101,std::vector<double>(joints));
        std::vector<double> vmax(joints),amax(joints);
        for(std::size_t j=0;j<joints;++j) {
            vmax[j]=.3+.05*j;amax[j]=.9+.1*j;
            for(std::size_t k=0;k<path.size();++k)
                path[k][j]=.2*j+(.12+.01*j)*std::sin(.073*k+.14*j)+.06*std::sin(.041*k+.2*j);
        }
        const auto original=path;
        TaughtMeasurementRoomRequest request;request.required_room_rad.assign(joints,.005);
        const auto schedule=plan_taught_measurement_schedule(path,request);
        check(schedule.valid && schedule.has_anchor,"fixture lacks real measured interior");
        const auto target=schedule.common_anchor.source_index;
        const auto outward=plan_taught_anchor_visit(path,.05,100,target,vmax,amax);
        const auto inward=plan_taught_anchor_visit(path,.05,target,100,vmax,amax);
        validate_visit(outward,vmax,amax);validate_visit(inward,vmax,amax);
        check(path==original,"original checked path was altered");
        close(outward.duration_s,inward.duration_s,"return timing excludes part of original path");
        close(outward.round_trip_cost_s,outward.duration_s+inward.duration_s,"round-trip cost lost return time");
        std::reverse(vmax.begin(),vmax.end());std::reverse(amax.begin(),amax.end());
        for(auto& q:path)std::reverse(q.begin(),q.end());
        const auto permuted=plan_taught_anchor_visit(path,.05,100,target,vmax,amax);
        close(permuted.duration_s,outward.duration_s,"joint permutation changed the algorithm");
        for(int k=0;k<=100;++k) {
            const auto a=sample_taught_anchor_visit(outward,k*outward.duration_s/100);
            const auto b=sample_taught_anchor_visit(permuted,k*permuted.duration_s/100);
            for(std::size_t j=0;j<joints;++j)close(a.position[j],b.position[joints-1-j],"joint identity affected path execution");
        }
        std::cout<<"anchor visit "<<joints<<" axes, anchor "<<target<<", round trip "<<outward.round_trip_cost_s<<" s PASS\n";
    }
}

void branches_and_budget() {
    // The duplicate .8 pose has two distinct approaches. A nearest-pose or
    // straight-line visit from index 4 to index 1 would omit the negative turn.
    const std::vector<std::vector<double>> path{{0},{.8},{-.2},{.8},{0},{.5}};
    const auto plan=plan_taught_anchor_visit(path,1,4,1,{2},{5});
    validate_visit(plan,{2},{5});
    double minimum=1,maximum=-1;
    for(int k=0;k<=10000;++k) {
        const auto s=sample_taught_anchor_visit(plan,plan.duration_s*k/10000);
        minimum=std::min(minimum,s.position[0]);maximum=std::max(maximum,s.position[0]);
    }
    check(minimum<-.199 && maximum>.799,"revisited pose branch was skipped");
    const double all_cost=plan.round_trip_cost_s+7.2+2;
    check(taught_anchor_visit_fits_budget(plan,all_cost,7.2,2),"exact complete cost should fit");
    check(!taught_anchor_visit_fits_budget(plan,all_cost-.001,7.2,2),"budget admission ignored return, measurement or exit time");
    check(!taught_anchor_visit_fits_budget(plan,all_cost,-1,2),"negative measurement cost was accepted");
    const auto constrained=plan_taught_anchor_visit(path,1,4,1,{.1},{.2});
    validate_visit(constrained,{.1},{.2});
    check(constrained.duration_s>plan.duration_s,"lower Safety capabilities were ignored");
    const auto very_short=plan_taught_anchor_visit({{0},{.001},{.002}},.001,0,1,{1},{1});
    validate_visit(very_short,{1},{1});
    const auto at_anchor=plan_taught_anchor_visit(path,1,2,2,{2},{5});
    check(at_anchor.valid && at_anchor.duration_s==0 && at_anchor.round_trip_cost_s==0,
          "already-present source anchor invented movement");
    const auto stationary=sample_taught_anchor_visit(at_anchor,0);
    check(stationary.complete && stationary.position==path[2] && stationary.velocity[0]==0 && stationary.acceleration[0]==0,
          "zero-cost anchor does not hold its original pose");
    check(!plan_taught_anchor_visit(path,0,0,1,{1},{1}).valid &&
          !plan_taught_anchor_visit(path,1,0,20,{1},{1}).valid &&
          !plan_taught_anchor_visit(path,1,0,1,{1,1},{1,1}).valid &&
          !plan_taught_anchor_visit(path,1,0,1,{1},{0}).valid,
          "invalid source mapping or capability accepted");
    TaughtMeasurementRoomRequest request;request.required_room_rad={.02};
    const auto schedule=plan_taught_measurement_schedule({{0},{.5},{1}},request);
    // Fresh .8 is useful although planned .5 was not reached. Arrival error
    // does not revoke valid actual room or create a new endpoint test gate.
    check(evaluate_taught_measurement_room({.8},schedule,request).bidirectional_measurable[0],
          "actual usable center was rejected by planned-arrival error");
}
}

int main() {
    try {
        arbitrary_axes_and_permutation();branches_and_budget();
        std::cout<<"Taught anchor visits: exact original branch, constrained derivatives, C2 rest clock, full budget PASS\n";
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
