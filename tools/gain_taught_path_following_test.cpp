#include "serial_arm/dynamics/gain_taught_path_following.hpp"
#include "serial_arm/dynamics/gain_measurement_center.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {
using namespace serial_arm::servo_tuning;
void require(bool value,const std::string& reason) {if(!value)throw std::runtime_error(reason);}
struct Run {double elapsed{0},minimum_rate{1},peak_acceleration{0};std::size_t limited{0},samples{0};};
Run follow(const std::vector<std::vector<double>>& path,double sample_dt,double lag,
    double max_rate,double lead,double deadline,bool quantized=false,
    double max_velocity=5.4,double max_acceleration=10.8) {
    const auto n=path.front().size();const double dt=.005,encoder=.0003814697265625;
    std::vector<double> actual=path.front(),previous(n,0),room(n,lead),reserve(n,3*encoder);
    std::vector<double> velocity(n,max_velocity),acceleration(n,max_acceleration);
    TaughtPathProgress progress;Run run;
    for(;run.elapsed<deadline;run.elapsed+=dt) {
        auto measured=actual;
        if(quantized)for(auto& q:measured)q=std::round(q/encoder)*encoder;
        const auto next=advance_taught_path(path,sample_dt,progress,measured,room,velocity,acceleration,dt,reserve,max_rate);
        require(next.valid,"taught follower rejected a supported plant: "+next.reason+" phase="+std::to_string(progress.source_time_s));
        require(next.progress.source_time_s+1e-12>=progress.source_time_s,"source phase moved backward");
        require(next.progress.source_time_s<=sample_dt*(path.size()-1)+1e-12,"source phase exceeded the demonstration");
        require(next.progress.source_time_s-progress.source_time_s<=max_rate*dt+1e-10,"phase advanced faster than requested");
        const auto original=serial_arm::calibration_retime::interpolate(path,next.progress.source_time_s/sample_dt,sample_dt,1);
        for(std::size_t i=0;i<n;++i) {
            require(std::abs(next.position[i]-original.position[i])<1e-12,"reference departed from original Hermite path");
            require(std::abs(next.velocity[i]-original.velocity[i]*next.progress.rate)<1e-12,"reference velocity is not the real phase derivative");
            require(std::abs(next.position[i]-measured[i])<=lead+1e-10,"feedback lead exceeded");
            require(std::abs(next.velocity[i])<=velocity[i]+1e-10,"velocity limit exceeded");
            const double a=std::abs(next.velocity[i]-previous[i])/dt;
            require(a<=acceleration[i]+1e-7,"acceleration limit exceeded");
            run.peak_acceleration=std::max(run.peak_acceleration,a);
            actual[i]+=(next.position[i]-actual[i])*(1-std::exp(-dt/lag));
        }
        previous=next.velocity;progress=next.progress;++run.samples;
        if(next.feedback_limited)++run.limited;
        if(progress.source_time_s>.1 && progress.source_time_s<sample_dt*(path.size()-1)-.1)
            run.minimum_rate=std::min(run.minimum_rate,progress.rate);
        if(next.complete) {
            require(progress.rate==0,"final phase did not brake to rest");
            require(next.position==path.back(),"final reference differs from taught endpoint");
            return run;
        }
    }
    throw std::runtime_error("continuous taught path did not complete inside its existing deadline; source="+
        std::to_string(progress.source_time_s)+" rate="+std::to_string(progress.rate));
}
std::vector<std::vector<double>> curved_path(std::size_t count,double duration) {
    constexpr double pi=3.14159265358979323846;
    std::vector<std::vector<double>> path(count,std::vector<double>(6));
    for(std::size_t k=0;k<count;++k) {
        const double t=static_cast<double>(k)/(count-1),a=2*pi*t;
        path[k]={.4*std::sin(a),.6*(1-std::cos(a)),.8*(1-std::cos(2*a)),
            1.1*std::sin(2*a),.6*std::sin(3*a),.6*std::sin(2.5*a)};
        // Small smooth deviations retain many genuine geometry nodes at the
        // measured half-code tolerance, while the continuous timebase remains
        // easy to execute. Stopping at every retained node wastes the budget.
        for(std::size_t i=0;i<6;++i)path[k][i]+=.002*std::sin((75+3*i)*a);
    }
    (void)duration;return path;
}
void many_nodes(const std::vector<std::vector<double>>& path,double sample_dt,bool historical) {
    std::vector<std::size_t> indices(path.size());std::iota(indices.begin(),indices.end(),0);
    const auto reduced=simplify_demonstrated_route(path,indices,std::vector<double>(path.front().size(),.0003814697265625));
    require(reduced.valid,"route reduction failed");
    const double geometric=demonstrated_path_transition_seconds(reduced.route,
        std::vector<double>(path.front().size(),.05),std::vector<double>(path.front().size(),.2));
    require(geometric>422.1,"regression fixture does not reproduce the per-node time problem: "+std::to_string(geometric)+" s");
    const auto run=follow(path,sample_dt,.035,.65,.04,422.1,true);
    require(run.minimum_rate>0,"the continuous follower stopped at a source knot");
    std::cout<<(historical?"recorded":"generated")<<" demonstration: nodes="<<path.size()<<" retained="<<reduced.route.size()
        <<" old_stop_at_nodes_s="<<geometric<<" continuous_s="<<run.elapsed<<" limited="<<run.limited<<'\n';
    if(historical) {
        const auto discovery=follow(path,sample_dt,.035,1,.04,422.1,true,1.62,3.24);
        std::cout<<"recorded discovery at 0.3 Safety, maximum phase rate 1: "<<discovery.elapsed
            <<" s; sampled acceleration "<<discovery.peak_acceleration<<" limited="<<discovery.limited<<'\n';
    }
}
void optional_recording(const char* filename) {
    std::ifstream input(filename);require(bool(input),"recording is missing");
    std::string line,field;std::getline(input,line);require(line.rfind("sample_dt,",0)==0,"invalid trajectory header");
    const double dt=std::stod(line.substr(10));std::getline(input,line);
    std::vector<std::vector<double>> path;
    while(std::getline(input,line)) {
        if(line.empty())continue;
        std::istringstream row(line);std::getline(row,field,',');
        std::vector<double> q;while(std::getline(row,field,','))q.push_back(std::stod(field));path.push_back(q);
    }
    many_nodes(path,dt,true);
}
void stopped_plant() {
    const std::vector<std::vector<double>> path{{0},{1}};
    TaughtPathProgress progress;double actual=0,previous=0;bool stopped=false,complete=false;
    for(std::size_t k=0;k<24000;++k) {
        const auto next=advance_taught_path(path,10,progress,{actual},{.04},{1.62},{3.24},.005,{.001145},1);
        require(next.valid,"stalled plant exceeded its reserved braking path: "+next.reason);
        require(std::abs(next.velocity[0]-previous)<=3.24*.005+1e-10,"stalled feedback caused abrupt reference freezing");
        if(k==1999) {
            stopped=next.feedback_limited && next.progress.rate<1e-7;
            require(next.position[0]<.04 && !next.complete,"stalled measurement was replaced by invented progress");
        }
        if(k>=2000)actual+=(next.position[0]-actual)*(1-std::exp(-.005/.035));
        previous=next.velocity[0];progress=next.progress;
        if(next.complete){complete=true;break;}
    }
    require(stopped && complete,"feedback stall did not brake and then resume the same taught path");
}
void soft_feedback_departure() {
    const std::vector<std::vector<double>> path{{0},{1}};
    constexpr double dt=.005,encoder=.0003814697265625;
    TaughtPathProgress progress;progress.source_time_s=5;progress.rate=.05;
    progress.previous_velocity={.15*progress.rate};
    // The measured position moves three original encoder codes away from an
    // otherwise legal reference. This is a soft following departure, not a
    // changed source path, limit or fabricated feedback sample.
    double actual=.5-.0393-3*encoder,previous=progress.previous_velocity[0];
    const auto strict=advance_taught_path(path,10,progress,{actual},{.04},{1.62},{3.24},dt,{3*encoder},.65);
    require(!strict.valid && strict.reason=="taught_path_reference_outside_feedback_lead",
        "candidate admission no longer preserves strict following checks");
    bool braking=false,waiting=false,recovered=false,complete=false;
    for(std::size_t k=0;k<24000;++k) {
        const auto next=advance_taught_path(path,10,progress,{actual},{.04},{1.62},{3.24},dt,{3*encoder},.65,false);
        require(next.valid,"three-code soft departure interrupted continuous discovery: "+next.reason);
        require(next.progress.source_time_s+1e-12>=progress.source_time_s,"soft recovery reversed the source phase");
        require(std::abs(next.velocity[0]-previous)<=3.24*dt+1e-10,"soft departure caused abrupt velocity freezing");
        const auto source=serial_arm::calibration_retime::interpolate(path,next.progress.source_time_s/10,10,1);
        require(std::abs(next.position[0]-source.position[0])<1e-12 &&
            std::abs(next.velocity[0]-source.velocity[0]*next.progress.rate)<1e-12,
            "soft recovery clipped or replaced the original q/qd");
        braking=braking || next.reason=="taught_path_braking_for_soft_feedback_recovery";
        waiting=waiting || next.reason=="taught_path_waiting_for_soft_feedback_recovery";
        recovered=recovered || next.reason=="taught_path_soft_feedback_recovered";
        if(k<60)require(!next.complete,"soft departure was reported as completion");
        else actual+=(next.position[0]-actual)*(1-std::exp(-dt/.035));
        previous=next.velocity[0];progress=next.progress;
        if(next.complete){complete=true;break;}
    }
    require(braking && waiting && recovered && complete,
        "soft feedback did not brake, wait, recover and complete the same taught path");
}
void soft_endpoint_quality_separation() {
    const std::vector<std::vector<double>> path{{0},{1}};
    const std::vector<double> actual{.8};
    TaughtPathProgress endpoint;endpoint.source_time_s=10;endpoint.previous_velocity={0};
    const auto strict=advance_taught_path(path,10,endpoint,actual,{.04},{1.62},{3.24},.005,{.001145},.65);
    require(!strict.valid,"strict endpoint unexpectedly accepted feedback outside its lead");
    const auto soft=advance_taught_path(path,10,endpoint,actual,{.04},{1.62},{3.24},.005,{.001145},.65,false);
    require(soft.valid && soft.complete && soft.feedback_limited && soft.position==path.back() && soft.velocity==std::vector<double>{0},
        "completed soft endpoint waited for a tracking-quality condition");
    require(soft.reason=="taught_path_reference_complete_with_feedback_warning" && actual[0]==.8,
        "soft reference completion erased or qualified the actual tracking error");
}
} // namespace

int main(int argc,char** argv) {
    try {
        std::vector<std::vector<double>> path(801,std::vector<double>(2));
        for(std::size_t k=0;k<path.size();++k) {
            const double t=static_cast<double>(k)/(path.size()-1);
            path[k]={.5*std::sin(3.14159265358979323846*t),.25*(1-std::cos(6.28318530717958647692*t))};
        }
        const auto fast=follow(path,.01,.02,.65,.025,100,true);
        const auto slow=follow(path,.01,.45,.65,.025,100,true);
        require(slow.limited>fast.limited,"feedback did not actually govern phase speed");
        require(slow.elapsed>fast.elapsed,"slower measured plant did not slow the reference");
        require(fast.minimum_rate>0,"follower stopped at intermediate source knots");
        std::cout<<"fast/slow plants: "<<fast.elapsed<<" / "<<slow.elapsed<<" s; sampled acceleration "
            <<fast.peak_acceleration<<" / "<<slow.peak_acceleration<<'\n';
        TaughtPathProgress empty;
        const auto bad=advance_taught_path(path,.01,empty,{1,1},{.01,.01},{1,1},{2,2},.005,{.001,.001});
        require(!bad.valid && bad.reason=="taught_path_reference_outside_feedback_lead","large feedback departure was silently clamped");
        require(bad.progress.source_time_s==0 && bad.progress.rate==0,"rejected step changed progress");
        const auto reserve=advance_taught_path(path,.01,empty,path.front(),{.01,.01},{1,1},{2,2},.005,{.01,.001});
        require(!reserve.valid,"invalid encoder reserve was accepted");
        const auto no_encoder=advance_taught_path(path,.01,empty,path.front(),{.01,.01},{1,1},{2,2},.005);
        require(no_encoder.valid,"unknown encoder incorrectly blocked ordinary demonstrated motion");
        stopped_plant();
        soft_feedback_departure();
        soft_endpoint_quality_separation();
        auto scale=path;for(auto& q:scale)q.push_back(q[0]*.5);
        follow(scale,.01,.05,.8,.03,100);
        const auto many=curved_path(2874,47.68429840740131);
        many_nodes(many,47.68429840740131/2873,false);
        if(argc>1)optional_recording(argv[1]);
        std::cout<<"continuous demonstrated-path checks passed\n";
        return EXIT_SUCCESS;
    } catch(const std::exception& error) {
        std::cerr<<"continuous demonstrated-path check failed: "<<error.what()<<'\n';return EXIT_FAILURE;
    }
}
