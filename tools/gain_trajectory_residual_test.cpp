// Uses the same residual and local-turn assessment called by production path
// tuning. Optional --record reads an existing CSV without changing its data.
#include "serial_arm/dynamics/gain_trajectory_tuning.hpp"
#include <fstream>
#include <functional>
#include <iostream>
#include <map>
#include <sstream>
using namespace serial_arm::servo_tuning;
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
std::vector<Sample> experiment(const std::function<double(double)>& reference,
    const std::function<double(double)>& actual,double end=12) {
    std::vector<Sample> rows;
    for(unsigned k=0;k<=static_cast<unsigned>((end+1.5)*200);++k) {
        Sample s;s.time_s=.005*k;s.reference=reference(std::min(s.time_s,end));s.actual=actual(std::min(s.time_s,end));
        s.reference_velocity=s.time_s<end?(reference(s.time_s+.0001)-reference(s.time_s-.0001))/.0002:0;
        s.velocity=s.time_s<end?(actual(s.time_s+.0001)-actual(s.time_s-.0001))/.0002:0;
        s.new_measurement=true;s.feedback_sequence=k+1;s.command_time_s=s.feedback_time_s=s.time_s;rows.push_back(s);
    }
    return rows;
}
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> values;std::stringstream input(line);std::string field;
    while(std::getline(input,field,','))values.push_back(field);
    return values;
}
void audit(const std::string& path) {
    std::ifstream file(path);if(!file)throw std::runtime_error("cannot read existing trajectory CSV");
    std::string line;std::getline(file,line);const auto names=split(line);std::map<std::string,std::size_t> index;
    for(std::size_t k=0;k<names.size();++k)index[names[k]]=k;
    std::map<std::string,std::vector<Sample>> axes;
    while(std::getline(file,line)) {
        const auto values=split(line);if(values.size()!=names.size())throw std::runtime_error("invalid existing trajectory row");
        const auto number=[&](const std::string& name){return std::stod(values.at(index.at(name)));};
        Sample s;s.time_s=number("time_s");s.reference=number("reference");s.reference_velocity=number("reference_velocity");
        s.actual=number("actual");s.velocity=number("velocity");s.new_measurement=number("new_measurement")!=0;
        s.limited=number("limited")!=0;axes[values.at(index.at("joint"))].push_back(s);
    }
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=.0003814697265625;
    for(const auto& [name,rows]:axes) {
        const double end=rows.back().time_s-req.tail_s;const auto a=assess_trajectory(rows,req,noise,end);
        const auto& motion=a.residual_motion;
        std::cout<<name<<" samples="<<rows.size()<<" peak_deg="<<a.metrics.peak_error*180/pi
            <<" tail_reversals="<<a.tail_position_reversals<<" uncommanded="<<motion.sustained_uncommanded
            <<" residual_hz="<<motion.frequency_hz<<" amplitude_deg="<<motion.amplitude_rad*180/pi
            <<" windows="<<motion.repeated_windows<<" commanded_component="<<motion.reference_component_present
            <<" local_overshoot_deg="<<a.local_reversal_overshoot_rad*180/pi<<'\n';
    }
}

void segment_candidates() {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;noise.velocity_rms=.007;
    const auto known=[&](std::vector<Sample> rows,double P=20,double D=.2) {
        for(auto& row:rows){row.actual_gains_known=true;row.actual_kp=P;row.actual_kd=D;row.feedback_time_s=row.command_time_s-.004;}
        return rows;
    };
    const auto forward=[](double t){return .04*t;};
    const auto lag=[](double t){return .04*t-.006;};
    auto measured=known(experiment(forward,lag,1.2));measured.resize(241);
    const auto segment=assess_trajectory_segment(measured,req,noise);
    check(segment.valid && segment.reference_direction==1 && !segment.uncommanded_reversals,
        "one complete one-way taught leg must be useful tuning evidence without a bidirectional probe");
    const Gains actual{{20},{.2}};
    const auto next=trajectory_segment_candidate_report(actual,{500},{5},{segment},req);
    check(next.changed && next.gains.kp[0]==21.6 && next.gains.kd==actual.kd,
        "one-way tracking deficit must propose a small independent stiffness comparison");
    const auto zero=[](double){return 0.;};const auto offset=[](double){return .01;};
    auto stalled=known(experiment(offset,zero,1.2),500.0/4095,.017);stalled.resize(241);
    const auto blocked=assess_trajectory_segment(stalled,req,noise);
    const auto one_code=trajectory_segment_candidate_report({{500.0/4095},{.017}},{500},{5},{blocked},req,{500.0/4095});
    check(one_code.changed && std::abs(one_code.gains.kp[0]-1000.0/4095)<1e-12 && one_code.gains.kd[0]==.017,
        "unresolved motion may request one actual protocol code without inventing higher damping or a hardware limit");
    auto ahead=known(experiment(forward,[](double t){return .04*t+.006;},1.2));ahead.resize(241);
    check(!trajectory_segment_candidate_report(actual,{500},{5},{assess_trajectory_segment(ahead,req,noise)},req).changed,
        "ahead-of-reference motion must not be treated as a stiffness deficit");
    const auto ripple=[](double t){return .002*std::sin(2*pi*40*t);};
    auto noisy=known(experiment(zero,ripple,1.2),20,5);noisy.resize(241);
    const auto unstable=assess_trajectory_segment(noisy,req,noise);
    check(unstable.uncommanded_reversals && unstable.velocity_noise_risk && unstable.position_reversals>=4,
        "an oscillatory window with zero net displacement must retain all its actual reversals and D-noise evidence");
    const auto quieter=trajectory_segment_candidate_report({{20},{5}},{500},{5},{unstable},req);
    check(quieter.gains.kp[0]==20 && quieter.gains.kd[0]==4.5,
        "velocity-dominated reversals must reduce D alone rather than compound P and D increases");
    auto quantized=known(experiment(zero,zero,1.2));quantized.resize(241);
    for(std::size_t i=0;i<quantized.size();++i){quantized[i].actual=(i%2?1:-1)*.5*noise.encoder_step_rad;quantized[i].velocity=(i%2?1:-1)*.007326;}
    const auto codes=assess_trajectory_segment(quantized,req,noise);
    check(!codes.uncommanded_reversals && !trajectory_segment_candidate_report(actual,{500},{5},{codes},req).changed,
        "one-code position chatter is neither repeated resolved oscillation nor evidence for a gain increase");
    auto blending=measured;blending.back().actual_kd=.3;
    check(!trajectory_segment_candidate_report(actual,{500},{5},{assess_trajectory_segment(blending,req,noise)},req).changed,
        "a changing applied controller cannot justify another segment gain revision");
    auto missing=measured;for(auto& row:missing)if(row.time_s>.6)row.time_s+=.1;
    check(!assess_trajectory_segment(missing,req,noise).valid,
        "a discontinuous path window must remain missing evidence");
    auto slow=measured;for(auto& row:slow)row.feedback_time_s=row.command_time_s-.04;
    check(!trajectory_segment_candidate_report(actual,{500},{5},{assess_trajectory_segment(slow,req,noise)},req).changed,
        "known delayed feedback must not authorize a higher feedback candidate");
    const auto duplicate=measured.front();auto repeated=std::vector<Sample>(300,duplicate);
    for(std::size_t i=0;i<repeated.size();++i)repeated[i].time_s=.005*i;
    check(!assess_trajectory_segment(repeated,req,noise).valid,
        "duplicate feedback sequence cannot manufacture a complete segment");
    const auto baseline=actual;
    for(std::size_t n:{1,3,7}) {
        Gains group;std::vector<TrajectorySegmentAssessment> segments;std::vector<double> pmax(n,500),dmax(n,5);
        for(std::size_t i=0;i<n;++i){group.kp.push_back(20);group.kd.push_back(.2);segments.push_back(segment);}
        const auto compared=trajectory_segment_candidate_report(group,pmax,dmax,segments,req);
        check(compared.gains.kp==std::vector<double>(n,21.6) && compared.gains.kd==group.kd,
            "segment tuning must scale to arbitrary mapped joint counts without joint-specific branches");
    }
    check(actual.kp==baseline.kp && actual.kd==baseline.kd,
        "generating or rejecting a candidate must preserve the complete measured parameter group");
    const auto one_way=assess_trajectory(known(experiment(forward,forward,12)),req,noise,12);
    check(one_way.passed && one_way.motion.qualified && !one_way.motion.bidirectional,
        "a complete accurate one-way demonstration must qualify its requested direction without claiming a reverse-direction measurement");
    const auto held=assess_trajectory(known(experiment(zero,zero,12)),req,noise,12);
    check(held.passed && held.stationary_reference && !held.motion.qualified && !held.responsive && !held.metrics.observable,
        "a stationary taught axis may satisfy its actual holding reference without inventing motion or identification");
    const auto poor=assess_trajectory(known(experiment(forward,lag,12)),req,noise,12);
    check(trajectory_trial_preferred(actual,{one_way},actual,{poor},actual,req),
        "complete one-way path improvement must remain comparable without a two-component linear fit");
    auto newly_noisy=one_way;newly_noisy.feedback_evidence.velocity_noise_risk=true;
    check(!trajectory_trial_preferred(actual,{newly_noisy},actual,{poor},actual,req),
        "a better raw tracking error cannot hide new velocity-feedback noise when retaining a complete candidate");
    std::cout<<"trajectory segments: one-way, stalled one-code drive, ahead motion, all-window reversals, D noise, quantization, blending, timing, duplicate feedback, 1/3/7 axes passed\n";
}

void final_target_acceptance() {
    Requirements req;req.settling_error_rad=.0005;NoiseEstimate noise;noise.encoder_step_rad=.0001;
    constexpr double duration=12;
    const auto measure=[&](double amplitude,double delay,double bias) {
        std::vector<Sample> rows;
        const auto path=[&](double t){return amplitude*.5*(1-std::cos(pi*std::clamp(t/duration,0.0,1.0)));};
        const auto velocity=[&](double t){return t>0 && t<duration?amplitude*.5*pi/duration*std::sin(pi*t/duration):0.;};
        for(unsigned k=0;k<=2700;++k) {
            Sample row;row.time_s=.005*k;row.reference=path(row.time_s);row.reference_velocity=velocity(row.time_s);
            row.actual=path(row.time_s-delay)+bias;row.velocity=velocity(row.time_s-delay);
            row.new_measurement=true;row.feedback_sequence=k+1;row.command_time_s=row.feedback_time_s=row.time_s;
            rows.push_back(row);
        }
        return rows;
    };
    // The path is deliberately late and misses a separately configured
    // settling band, yet its final half-second positioning window is accurate.
    for(std::size_t count:{1,3,7})for(std::size_t axis=0;axis<count;++axis) {
        const auto rows=measure(.3+.02*axis,1.0,.0008);
        const auto result=assess_trajectory(rows,req,noise,duration);const auto& m=result.metrics;
        check(result.passed && m.endpoint_window_available && m.endpoint_passed &&
            m.endpoint_measurement_samples>=100 && m.endpoint_peak_error_rad<req.peak_error_rad &&
            m.peak_error>req.peak_error_rad && m.rms_error>req.rms_error_rad && m.settling_s>req.settling_time_s,
            "late but smooth complete trajectories must score only the measured final-target error for generic axis counts");
        check(std::abs(m.endpoint_window_end_s-m.endpoint_window_start_s-.5)<1e-12 &&
            std::abs(m.endpoint_bias_error_rad-.0008)<1e-12 && m.endpoint_rms_error_rad>.00079,
            "endpoint score must retain its actual window, fresh count, signed bias, RMS and raw maximum");
    }
    const auto complete=measure(.3,1.0,.0008);
    auto missing_tail=complete;missing_tail.resize(2401);
    const auto missing=assess_trajectory(missing_tail,req,noise,duration);
    check(!missing.passed && !missing.metrics.endpoint_window_available,
        "a trajectory without a final dwell must retain missing evidence rather than a zero endpoint error pass");
    auto partial_tail=complete;partial_tail.resize(2601);
    check(!assess_trajectory(partial_tail,req,noise,duration).metrics.endpoint_window_available,
        "an interrupted endpoint dwell cannot silently shorten the required final measurement");
    auto still_moving=complete;
    for(auto& row:still_moving)if(row.time_s>=13){row.reference+=.001*(row.time_s-13);row.reference_velocity=.001;}
    check(!assess_trajectory(still_moving,req,noise,duration).metrics.endpoint_window_available,
        "a small moving reference is not a fixed final target regardless of its tracking error");
    auto repeated=complete;
    for(auto& row:repeated)if(row.time_s>=13)row.feedback_sequence=2601;
    check(!assess_trajectory(repeated,req,noise,duration).metrics.endpoint_window_available,
        "repeated feedback marked fresh cannot manufacture a final-target window");
    auto unfresh=complete;
    for(auto& row:unfresh)if(row.time_s>=13)row.new_measurement=false;
    check(!assess_trajectory(unfresh,req,noise,duration).metrics.endpoint_window_available,
        "a held last position without fresh feedback cannot qualify positioning");
    auto stale=complete;
    for(auto& row:stale)if(row.time_s>=13)row.feedback_time_s=row.command_time_s-.1;
    check(!assess_trajectory(stale,req,noise,duration).metrics.endpoint_window_available,
        "stale command/feedback timing must remain missing endpoint evidence");
    auto limited=complete;limited.back().limited=true;
    check(!assess_trajectory(limited,req,noise,duration).metrics.endpoint_window_available,
        "a clipped endpoint is not an accepted fixed-target positioning measurement");
    auto cancelled_mean=complete;
    for(auto& row:cancelled_mean)if(row.time_s>=13)row.actual=row.reference+(row.time_s<13.25?-.002:.002);
    const auto spike=assess_trajectory(cancelled_mean,req,noise,duration);
    check(spike.metrics.endpoint_window_available && !spike.metrics.endpoint_passed && !spike.passed &&
        std::abs(spike.metrics.endpoint_bias_error_rad)<.0001 && spike.metrics.endpoint_peak_error_rad>.0019,
        "opposite signed endpoint errors cannot average into a positioning pass");
    auto boundary=assess_trajectory(complete,req,noise,duration);
    auto exact=req;exact.peak_error_rad=boundary.metrics.endpoint_peak_error_rad;
    auto just_above=exact;just_above.peak_error_rad=std::nextafter(exact.peak_error_rad,std::numeric_limits<double>::infinity());
    check(!assess_trajectory(complete,exact,noise,duration).passed && assess_trajectory(complete,just_above,noise,duration).passed,
        "the final-target score must preserve the requested strict less-than accuracy boundary");
    auto bad_endpoint=measure(.3,.2,.02);
    const auto poor=assess_trajectory(bad_endpoint,req,noise,duration);
    check(poor.metrics.endpoint_window_available && !poor.passed && trajectory_measurement_complete(bad_endpoint,duration,req.tail_s) &&
        trajectory_tuning_measurement(poor),
        "an inaccurate final-target measurement remains complete and usable for further tuning without terminating the workflow");
    std::cout<<"final target acceptance: late full paths, 1/3/7 axes, raw endpoint peaks, strict score boundary, missing/partial/moving/stale/repeated evidence passed\n";
}

void budgeted_whole_group_comparisons() {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;
    const double pcode=500.0/4095,dcode=5.0/4095;
    ModelAxisLimits limit;limit.kp_max=500;limit.kd_max=5;limit.effort_budget=20;limit.grid={pcode,dcode};
    const auto measure=[&](double P,double D) {
        auto rows=experiment([](double t){return .04*t;},[](double t){return .04*t-.03;},12);
        for(auto& row:rows){row.actual_gains_known=true;row.actual_kp=P;row.actual_kd=D;row.feedback_time_s=row.command_time_s-.004;}
        return assess_trajectory(rows,req,noise,12);
    };
    const Gains small{{3*pcode},{5}};const auto complete=measure(small.kp[0],small.kd[0]);
    check(complete.responsive && complete.feedback_evidence.valid && complete.feedback_evidence.timing_known,
        "the local comparison fixture must use a complete actually observed path response");
    const auto p=budgeted_trajectory_local_candidate(small,{limit},{complete},req);
    check(p.changed && gain_grid_code(p.gains.kp[0],pcode,500)==4 && p.gains.kd==small.kd,
        "a sub-code percentage P trial must become one real code while capped D remains unchanged");
    check(p.reasons[0].find("candidate_comparison_not_identification")!=std::string::npos,
        "a local comparison must not claim that the plant or bandwidth has been identified");
    check(p.gains.kp[0]<=1.5*small.kp[0] &&
        complete.metrics.peak_error*(1-small.kp[0]/p.gains.kp[0])>=3*complete.feedback_evidence.position_floor_rad,
        "the bounded quasi-static trial assumption must predict an encoder-resolvable comparison");
    auto unresolved=complete;unresolved.metrics.peak_error=unresolved.feedback_evidence.peak_error_rad=
        5*complete.feedback_evidence.position_floor_rad;
    check(!budgeted_trajectory_local_candidate(small,{limit},{unresolved},req).changed,
        "a resolvable change requiring over fifty percent stiffness is outside one local comparison");
    auto noise_risk=complete;noise_risk.feedback_evidence.velocity_noise_risk=true;
    const Gains tiny_d{{3*pcode},{dcode}};
    const auto d=budgeted_trajectory_local_candidate(tiny_d,{limit},{noise_risk},req);
    check(d.changed && d.gains.kp==tiny_d.kp && gain_grid_code(d.gains.kd[0],dcode,5)==0,
        "a sub-code noise-related damping reduction must change one actual D code alone");
    auto ringing=complete;ringing.metrics.oscillating=true;
    const auto lower=budgeted_trajectory_local_candidate({{pcode},{0}},{limit},{ringing},req);
    check(lower.changed && lower.gains.kp[0]==0 && lower.gains.kd[0]==0,
        "one-code P reductions may reach the protocol zero boundary without negative gains");
    const auto zero_d=budgeted_trajectory_local_candidate({{pcode},{0}},{limit},{noise_risk},req);
    check(!zero_d.changed,"a damping reduction at zero cannot manufacture a negative or unchanged-code candidate");
    check(!budgeted_trajectory_local_candidate({{500},{5}},{limit},{complete},req).changed,
        "a stiffness increase at its protocol maximum must remain unavailable");
    for(std::size_t n:{1,3,7}) {
        Gains current;current.kp.assign(n,3*pcode);current.kd.assign(n,5);
        std::vector<TrajectoryAssessment> axes(n,complete);const auto before=current;
        const auto worst=n-1;axes[worst].metrics.peak_error*=2;
        const auto candidate=budgeted_trajectory_local_candidate(current,std::vector<ModelAxisLimits>(n,limit),axes,req);
        check(candidate.changed && candidate.gains.kd==current.kd,"the arbitrary-axis comparison must preserve complete damping");
        for(std::size_t i=0;i<n;++i)check(candidate.gains.kp[i]==4*pcode,
            "every supported axis must participate in the complete budgeted comparison, independent of worst-axis rank");
        std::reverse(axes.begin(),axes.end());
        const auto permuted=budgeted_trajectory_local_candidate(current,std::vector<ModelAxisLimits>(n,limit),axes,req);
        check(permuted.gains.kp[n-1-worst]==4*pcode,"whole-group candidate must follow mapped evidence after permutation");
        check(current.kp==before.kp && current.kd==before.kd,"candidate generation cannot mutate the complete measured incumbent");
    }
    for(unsigned fault=0;fault<9;++fault) {
        auto missing=complete;
        switch(fault) {
            case 0:missing.metrics.measurement_samples=0;break;
            case 1:missing.metrics.limited_samples=1;break;
            case 2:missing.feedback_evidence.limited=true;break;
            case 3:missing.feedback_evidence.gains_constant=false;break;
            case 4:missing.feedback_evidence.timing_known=false;break;
            case 5:missing.feedback_evidence.feedback_age_max_s=.04;break;
            case 6:missing.feedback_evidence.sample_interval_max_s=.04;break;
            case 7:missing.responsive=false;break;
            case 8:missing.feedback_evidence.velocity_error_peak_rad_s=std::numeric_limits<double>::quiet_NaN();break;
        }
        check(!budgeted_trajectory_local_candidate(small,{limit},{missing},req).changed,
            "incomplete, limited, stale, unresolved or unknown actual feedback cannot authorize a local experiment");
    }
    auto large_error=complete;large_error.metrics.peak_error=.4;large_error.feedback_evidence.peak_error_rad=.4;
    large_error.feedback_evidence.velocity_error_rms_rad_s=.01;
    large_error.feedback_evidence.velocity_error_peak_rad_s=.5;
    auto restricted=limit;restricted.effort_budget=small.kp[0]*.4+small.kd[0]*.5;
    check(!budgeted_trajectory_local_candidate(small,{restricted},{large_error},req).changed,
        "full measured position and velocity error peaks must bound effort even when probe amplitude and velocity RMS are small");
    restricted.effort_budget=4*pcode*.4+small.kd[0]*.5+1e-9;
    const auto admitted=budgeted_trajectory_local_candidate(small,{restricted},{large_error},req);
    check(admitted.changed && admitted.gains.kp[0]==4*pcode && admitted.gains.kd==small.kd,
        "an exact one-code effort allowance should admit P without silently reducing the measured D");
    auto measured=experiment([](double t){return .02*t;},[](double t){return .02*t-.006;},1.2);
    for(auto& row:measured){row.actual_gains_known=true;row.actual_kp=1;row.actual_kd=.1;}
    measured[120].velocity=1.02;
    const auto velocity=assess_trajectory_segment(measured,req,noise);
    check(std::abs(velocity.velocity_error_peak_rad_s-1)<1e-6 && velocity.velocity_error_rms_rad_s<.1,
        "the trajectory evidence must retain a measured velocity-error peak separately from its RMS");
    std::cout<<"PASS budgeted whole-group path comparison: real P/D codes, capped D, 1/3/7 mapping, full-error effort, zero/max bounds and incomplete feedback\n";
}

// Explicit offline plant: sampled MIT feedback at 200 Hz, 5 ms delayed torque,
// Coulomb/viscous friction, 16-bit position and 12-bit velocity feedback. These
// declared plant assumptions are not estimates of the user's motor inner loop.
std::vector<Sample> taught_plant(double P,double D,double inertia) {
    constexpr double dt=.001,end=16,position_code=25.0/65536,velocity_code=20.0/4095;
    constexpr unsigned sample_period=5,delay=5;
    std::vector<double> commands(delay,0);std::vector<Sample> rows;double q=0,v=0,command=0;
    for(unsigned k=0;k<=static_cast<unsigned>((end+1.5)/dt);++k) {
        const double time=k*dt;
        const double r=time<end?.08*std::sin(2*pi*time/16):0;
        const double rv=time<end?.08*(2*pi/16)*std::cos(2*pi*time/16):0;
        const double measured=std::round(q/position_code)*position_code;
        const double velocity=std::round(v/velocity_code)*velocity_code;
        if(k%sample_period==0)command=P*(r-measured)+D*(rv-velocity);
        const double applied=commands[k%delay];commands[k%delay]=command;
        const double friction=(inertia/.2)*(.1*std::tanh(v/.001)+.08*v);
        v+=(applied-friction)/inertia*dt;q+=v*dt;
        if(k%sample_period)continue;
        Sample row;row.time_s=time;row.reference=r;row.reference_velocity=rv;row.actual=measured;row.velocity=velocity;
        row.actual_gains_known=true;row.actual_kp=P;row.actual_kd=D;row.new_measurement=true;row.feedback_sequence=k+1;
        row.command_time_s=time;row.feedback_time_s=time-.005;row.feedforward_known=true;row.applied_feedforward_nm=0;rows.push_back(row);
    }
    return rows;
}

void reproducible_measured_path_comparisons() {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;noise.velocity_rms=.002;
    const double pcode=500.0/4095,dcode=5.0/4095;
    const std::vector<double> inertia{.2,.035};Gains gains{{4,.7},{1.4,.245}};const Gains seed=gains;
    ModelAxisLimits limit;limit.kp_max=500;limit.kd_max=5;limit.effort_budget=5;limit.grid={pcode,dcode};
    const std::vector<ModelAxisLimits> limits(inertia.size(),limit);
    const auto measure=[&](const Gains& candidate) {
        std::vector<TrajectoryAssessment> measured;
        for(std::size_t i=0;i<inertia.size();++i)measured.push_back(assess_trajectory(taught_plant(candidate.kp[i],candidate.kd[i],inertia[i]),req,noise,16));
        return measured;
    };
    auto measured=measure(gains);const auto initial=measured;unsigned kept=0,rejected=0;
    for(unsigned trial=0;trial<5;++trial) {
        const auto proposal=budgeted_trajectory_local_candidate(gains,limits,measured,req);
        std::cout<<"offline baseline reason="<<measured[0].reason<<", responsive="<<measured[0].responsive
            <<", tvalid="<<measured[0].tuning.valid<<", gain="<<measured[0].tuning.in_phase_gain
            <<", lag="<<measured[0].tuning.lag_s<<", A/V="<<measured[0].tuning.amplitude_error_rms_rad
            <<'/'<<measured[0].tuning.velocity_error_rms_rad<<", candidate="<<proposal.reasons[0]<<'\n';
        check(proposal.changed,"a resolved frictional demonstrated-path deficit must generate a measured comparison");
        std::size_t changed_axes=0;
        for(std::size_t i=0;i<inertia.size();++i) {
            if(proposal.gains.kp[i]!=gains.kp[i] || proposal.gains.kd[i]!=gains.kd[i])++changed_axes;
            check(proposal.gains.kd[i]==gains.kd[i],"the measured deficit experiment must preserve independent damping");
        }
        check(changed_axes>0,"each budgeted traversal must compare a changed whole group without requiring one-axis-at-a-time trials");
        const auto compared=measure(proposal.gains);
        const bool preferred=trajectory_trial_preferred(proposal.gains,compared,gains,measured,seed,req);
        std::cout<<"offline path trial "<<trial+1<<" P/D="<<proposal.gains.kp[0]<<'/'<<proposal.gains.kd[0]
            <<" peak="<<compared[0].metrics.peak_error<<" rms="<<compared[0].metrics.rms_error<<" kept="<<preferred
            <<" reason="<<proposal.reasons[0]<<'\n';
        if(preferred){gains=proposal.gains;measured=compared;++kept;}else{++rejected;break;}
    }
    check(kept>0 && measured[0].metrics.peak_error<.95*initial[0].metrics.peak_error &&
          measured[0].metrics.rms_error<.95*initial[0].metrics.rms_error,
        "actual measured complete-set selection must improve both full-cycle peak and RMS on a repeatable delayed/frictional/quantized MIT plant");
    const auto healthy=gains;auto harmful=gains;harmful.kp[0]*=.25;harmful.kp[1]*=4;
    const auto unsafe=measure(harmful);
    check(!trajectory_trial_preferred(harmful,unsafe,gains,measured,seed,req),
        "improvement on another measured joint must not hide one worsened axis when retaining a complete controller");
    ++rejected;check(gains.kp==healthy.kp && gains.kd==healthy.kd && gains.kd==seed.kd,
        "a rejected experiment must retain the exact complete measured group, with no implicit D escalation or stitched joint mix");
    std::cout<<"PASS repeatable complete taught-path trials: kept="<<kept<<", rejected="<<rejected<<", peak "
        <<initial[0].metrics.peak_error<<" -> "<<measured[0].metrics.peak_error<<", RMS "
        <<initial[0].metrics.rms_error<<" -> "<<measured[0].metrics.rms_error<<" rad\n";
}

std::vector<Sample> delayed_underdamped_mit(double P,double D) {
    // Declared offline plant: 30 g m^2 effective inertia, 200 Hz sampled MIT
    // command, 4 ms torque delay, Coulomb/viscous friction and quantized
    // feedback. The initial displacement excites a physical decaying mode;
    // the smooth reference itself does not request that mode.
    constexpr double dt=.001,end=12,I=.03,qcode=25.0/65536,vcode=20.0/4095;
    constexpr unsigned delay=4;std::vector<double> commands(delay,0);std::vector<Sample> result;
    double q=.02,v=0,command=0;
    for(unsigned k=0;k<=static_cast<unsigned>((end+1.5)/dt);++k) {
        const double time=k*dt,w=2*pi/4;
        const double reference=time<end?.03*std::sin(w*time):0,reference_velocity=time<end?.03*w*std::cos(w*time):0;
        const double measured=std::round(q/qcode)*qcode,velocity=std::round(v/vcode)*vcode;
        if(k%5==0)command=P*(reference-measured)+D*(reference_velocity-velocity);
        const double torque=commands[k%delay];commands[k%delay]=command;
        const double friction=.003*std::tanh(v/.005)+.006*v;
        v+=(torque-friction)/I*dt;q+=v*dt;
        if(k%5)continue;
        Sample row;row.time_s=time;row.reference=reference;row.reference_velocity=reference_velocity;row.actual=measured;row.velocity=velocity;
        row.new_measurement=true;row.feedback_sequence=k+1;row.actual_gains_known=true;row.actual_kp=P;row.actual_kd=D;
        row.command_time_s=time;row.feedback_time_s=time-.004;row.feedforward_known=true;row.applied_feedforward_nm=0;
        result.push_back(row);
    }
    return result;
}

void measured_independent_damping_comparison() {
    Requirements req;NoiseEstimate noise;noise.encoder_step_rad=25.0/65536;noise.velocity_rms=.002;
    const Gains baseline{{20},{.15}};const auto before_rows=delayed_underdamped_mit(20,.15);
    const auto before=assess_trajectory(before_rows,req,noise,12);
    const auto proposal=trajectory_damping_candidate_report(baseline,{500},{5},{.03},{before},req,{5.0/4095});
    check(before.residual_motion.sustained_uncommanded && proposal.changed && proposal.gains.kp==baseline.kp &&
        proposal.gains.kd[0]>baseline.kd[0] && proposal.gains.kd[0]<.25 &&
        proposal.gains.kd[0]<=proposal.timing_damping_ceiling[0],
        "resolved repeated residual with low modeled damping must generate one bounded independent D comparison, preserving P");
    const auto after=assess_trajectory(delayed_underdamped_mit(20,proposal.gains.kd[0]),req,noise,12);
    check(after.metrics.rms_error<.95*before.metrics.rms_error &&
        after.metrics.velocity_rms<.95*before.metrics.velocity_rms &&
        after.metrics.peak_error<=before.metrics.peak_error+noise.encoder_step_rad,
        "the generated damping comparison must actually improve error and motion smoothness on the declared delayed/frictional MIT plant");
    check(trajectory_trial_preferred(proposal.gains,{after},baseline,{before},baseline,req),
        "the complete measured damping group must be retained only after an actual same-path improvement");
    auto overshoot=before;overshoot.residual_motion.sustained_uncommanded=false;
    check(!trajectory_damping_candidate_report(baseline,{500},{5},{.03},{overshoot},req).changed,
        "one dynamic overshoot without repeated nonreference residual is not evidence for a damping increase");
    auto commanded=before;commanded.residual_motion.reference_amplitude_rad=commanded.residual_motion.amplitude_rad;
    check(!trajectory_damping_candidate_report(baseline,{500},{5},{.03},{commanded},req).changed,
        "a reference-commanded fast component must not be treated as an uncommanded low-damping mode");
    auto delayed=before;delayed.feedback_evidence.feedback_age_max_s=.04;
    check(!trajectory_damping_candidate_report(baseline,{500},{5},{.03},{delayed},req).changed,
        "a stale actual feedback path must withhold a larger D candidate");
    auto noisy=before;noisy.feedback_evidence.velocity_noise_risk=true;
    const auto reduction=trajectory_damping_candidate_report(baseline,{500},{5},{.03},{noisy},req);
    check(reduction.changed && reduction.gains.kp==baseline.kp && reduction.gains.kd[0]<baseline.kd[0],
        "velocity-noise-dominated feedback must reduce D, not follow a low nominal ratio toward higher damping");
    auto unavailable=before;unavailable.feedback_evidence.gains_constant=false;
    check(!trajectory_damping_candidate_report(baseline,{500},{5},{.03},{unavailable},req).changed,
        "an unresolved actual-gain handover cannot justify another damping revision");
    auto tiny=before;tiny.residual_motion.amplitude_rad=tiny.residual_motion.position_floor_rad;
    check(!trajectory_damping_candidate_report(baseline,{500},{5},{.03},{tiny},req).changed,
        "sub-resolution residual cannot justify a damping increase");
    const auto constrained=trajectory_damping_candidate_report(baseline,{500},{5},{.001},{before},req);
    check(!constrained.changed && constrained.timing_damping_ceiling[0]<baseline.kd[0],
        "a free-inertia/timing experiment bound cannot be bypassed by a low nominal damping ratio");
    for(std::size_t n:{1,3,7}) {
        Gains group;group.kp.assign(n,20);group.kd.assign(n,.15);
        const auto candidate=trajectory_damping_candidate_report(group,std::vector<double>(n,500),std::vector<double>(n,5),
            std::vector<double>(n,.03),std::vector<TrajectoryAssessment>(n,before),req,std::vector<double>(n,5.0/4095));
        check(candidate.changed && candidate.gains.kp==group.kp && candidate.gains.kd.size()==n,
            "model/timing damping candidates must support arbitrary mapped joint counts without joint-specific branches");
    }
    std::cout<<"PASS independent D comparison P="<<baseline.kp[0]<<" D="<<baseline.kd[0]<<" -> "<<proposal.gains.kd[0]
        <<" RMS="<<before.metrics.rms_error<<" -> "<<after.metrics.rms_error
        <<" velocity RMS="<<before.metrics.velocity_rms<<" -> "<<after.metrics.velocity_rms<<'\n';
}
}
int main(int argc,char** argv) {
    try {
        if(argc==3 && std::string(argv[1])=="--record"){audit(argv[2]);return 0;}
        segment_candidates();final_target_acceptance();budgeted_whole_group_comparisons();reproducible_measured_path_comparisons();measured_independent_damping_comparison();Requirements req;NoiseEstimate noise;noise.encoder_step_rad=.0003814697265625;
        const auto smooth=[](double t){return .15*std::sin(2*pi*.25*t);};
        const auto ripple=[&](double t){return smooth(t)+(t>=1 && t<10?.008*std::sin(2*pi*8*t):0);};
        const auto vibrating=experiment(smooth,ripple);
        const auto a=assess_trajectory(vibrating,req,noise,12);
        check(a.residual_motion.valid && a.residual_motion.sustained_uncommanded && a.metrics.oscillating,
            "motion-window ripple must survive a quiet final tail");
        check(a.tail_position_reversals==0 && std::abs(a.residual_motion.frequency_hz-8)<.6,
            "position residual must identify repeated motion-window frequency without inventing tail motion");
        check(!a.passed && a.metrics.endpoint_passed && a.metrics.peak_error>=.0079,
            "an accurate final target must not hide measured sustained oscillation or change the raw dynamic peak");
        const auto commanded=[](double t){return .01*std::sin(2*pi*8*t)+.1*std::sin(2*pi*.2*t);};
        const auto commanded_delay=[&](double t){return commanded(t-.02);};
        const auto c=assess_trajectory(experiment(commanded,commanded_delay),req,noise,12);
        check(!c.residual_motion.sustained_uncommanded && c.residual_motion.reference_component_present,
            "a commanded high-frequency reference with delay is not uncommanded oscillation");
        const auto lag=[&](double t){return smooth(t-.04);};
        check(!assess_trajectory(experiment(smooth,lag),req,noise,12).metrics.oscillating,
            "ordinary low-frequency tracking lag must remain separate from sustained oscillation");
        const auto quantized=[&](double t){return std::round(smooth(t)/noise.encoder_step_rad)*noise.encoder_step_rad+
            .3*noise.encoder_step_rad*std::sin(2*pi*11*t);};
        check(!assess_trajectory(experiment(smooth,quantized),req,noise,12).metrics.oscillating,
            "encoder steps and sub-noise jitter must not trigger a ripple diagnosis");
        const auto brief=[&](double t){return smooth(t)+(t>=3 && t<3.3?.01*std::sin(2*pi*8*t):0);};
        check(!assess_trajectory(experiment(smooth,brief),req,noise,12).residual_motion.sustained_uncommanded,
            "one short transient must not be reported as sustained oscillation");
        const auto local_reference=[](double t){
            if(t<1)return .05*t;
            if(t<2)return .05*(2-t);
            if(t<4)return .1*(t-2);
            if(t<6)return .2-.1*(t-4);
            return 0.;};
        const auto local_actual=[&](double t){return local_reference(t)+(t>=1 && t<1.25?.015*std::sin(pi*(t-1)/.25):0);};
        const auto local=assess_trajectory(experiment(local_reference,local_actual,8),req,noise,8);
        check(local.metrics.overshoot<1e-9 && local.local_reversal_overshoot_rad>.006 &&
            local.dynamic_overshoot_rad>.006 && !local.residual_motion.sustained_uncommanded && local.passed,
            "one local-turn overshoot remains a dynamic diagnostic while the accurate final target qualifies positioning");
        const auto perfect_local=experiment(local_reference,local_reference,8);
        check(trajectory_local_reversal_overshoot(perfect_local,8,.001,-.005,.7)==0,
            "a final load bias of the opposite sign must not invent an earlier local overshoot");
        const Gains gains{{40},{.15}};const auto candidate=trajectory_candidate_report(gains,{50},{.2},{a},req);
        check(candidate.changed && std::abs(candidate.gains.kp[0]-36.8)<1e-12 && candidate.gains.kd[0]==.15,
            "uncommanded oscillation must lower stiffness while preserving measured damping without an identified delay");
        check(candidate.gains.kp[0]<=50 && candidate.gains.kd[0]<=.2,
            "ripple-based candidates must retain the existing bounds");
        auto repeated=vibrating;for(auto& row:repeated)row.new_measurement=false;
        check(!trajectory_residual_motion(repeated,noise,12,req.settling_time_s).valid,
            "repeated feedback cannot become measured oscillation evidence");
        auto interrupted=vibrating;for(auto& row:interrupted)if(row.time_s>6)row.time_s+=.1;
        check(!trajectory_residual_motion(interrupted,noise,12,req.settling_time_s).valid,
            "a feedback gap must retain insufficient evidence rather than produce a spectral conclusion");
        std::cout<<"trajectory residual: moving ripple, commanded frequency, delay, quantization, one transient, local overshoot, bounded P/D, repeated/gapped feedback passed\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}return 0;
}
