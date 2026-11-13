#include <serial_arm/dynamics/servo_gain_tuning.hpp>
#include <serial_arm/dynamics/gain_disturbance_plan.hpp>
#include <serial_arm/dynamics/gain_frequency_plan.hpp>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <stdexcept>

using namespace serial_arm::servo_tuning;
namespace {
void check(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
Sample sample(std::size_t k,double actual,double reference=0) {
    Sample row;row.time_s=.005*k;row.actual=actual;row.reference=reference;
    row.new_measurement=true;row.feedback_sequence=k+1;return row;
}
void synthetic_checks() {
    constexpr double step=.0003814697265625;
    ObservedPositionStep codes;
    for(std::size_t k=0;k<20;++k)codes.observe(k%2?.1:0,k+1);
    check(codes.qualified_step()==0,"two positions or a repeated large jump do not identify encoder resolution");
    codes.observe(.10001,21);
    check(codes.qualified_step()==0,"one isolated small difference is not repeated encoder-code evidence");
    ObservedPositionStep lattice;
    for(std::size_t k=0;k<10;++k)lattice.observe(.10002+(k%3)*.00001,k+1);
    check(lattice.qualified_step()>0 && lattice.step_evidence_pairs()>=2,"repeated adjacent code spacings establish observable resolution");

    for(const std::size_t axes:{1u,3u,7u}) {
        std::vector<CommandConditionedMotionGuard> guards(axes);
        std::vector<bool> tripped(axes,false);MotionGuardResult learned;
        for(std::size_t k=0;k<800;++k)for(std::size_t axis=0;axis<axes;++axis) {
            // A stationary one-code baseline cannot identify the step. Later
            // slow motion provides three codes; its small chatter is raw.
            const double ref=k<60?0:.0008*(k-60)*.005;
            const double q=k<60?0:step*std::round((ref+step*std::sin(2*pi*39*.005*k))/step);
            auto row=sample(k,q,ref);row.velocity=100*std::sin(2*pi*70*row.time_s);
            const auto result=guards[axis].observe(row);tripped[axis]=tripped[axis] || result.triggered;learned=result;
        }
        check(std::none_of(tripped.begin(),tripped.end(),[](bool value){return value;}),"one-code chatter after an uninformative quiet baseline must not trip for any joint count");
        check(learned.encoder_step_known && std::abs(learned.effective_encoder_step_rad-step)<1e-10 && learned.fresh_position_codes>=3,
            "the guard must learn repeated raw codes online before the trajectory finishes");
    }

    for(const bool known:{false,true})for(unsigned kind=0;kind<9;++kind) {
        NoiseEstimate noise;if(known)noise.encoder_step_rad=step;
        CommandConditionedMotionGuard guard({},noise);bool triggered=false;MotionGuardResult latest;
        for(std::size_t k=0;k<800;++k) {
            const double t=.005*k;double ref=0,q=0;
            if(kind==0){ref=.012*std::sin(2*pi*12*t);q=ref;}
            if(kind==1)q=.04*t*std::exp(-8*t); // One overshoot, then recovery.
            if(kind==2){ref=.003*t;q=ref+.009*std::sin(2*pi*25*t);}
            if(kind==3){q=.015*(static_cast<int>(k%3)-1);} // Even repeated three-code large jumps cannot become a quantum.
            if(kind==4){q=step*std::sin(2*pi*70*t);}
            if(kind==5){q=(t<.3?0:.04)+2*step*std::sin(2*pi*70*t);}
            if(kind==6){q=.4*t*std::exp(-8*t)+2*step*std::sin(2*pi*70*t);}
            if(kind==7){q=.009*(1+.5*t)*std::sin(2*pi*25*t);}
            if(kind==8){q=.02*std::exp(-.5*t)*std::sin(2*pi*25*t);}
            latest=guard.observe(sample(k,q,ref));triggered=triggered || latest.triggered;
        }
        check(triggered==(kind==2 || kind==3 || kind==7 || kind==8),
            "one large transition/overshoot plus code chatter must stay distinct from repeated large excursions, including gradual growth/decay");
        if(kind==3 && !known)check(!latest.encoder_step_known && latest.hysteresis_rad<=.002+1e-12,
            "repeated large sampled motion increments must not identify an encoder quantum or enlarge unknown-resolution protection");
        if(kind==4 && known)check(latest.effective_encoder_step_rad==step,
            "equal small analog increments near sine turning points must not shrink a known encoder quantum");
    }

    CommandConditionedMotionGuard guard;
    MotionGuardResult before;
    for(std::size_t k=0;k<50;++k)before=guard.observe(sample(k,step*(k%3)));
    check(before.encoder_step_known,"duplicate-frame test needs a resolved initial trace");
    for(std::size_t k=0;k<100;++k) {
        auto row=sample(49,.2*((k%2)?1:-1));row.feedback_sequence=10+k%2;
        const auto result=guard.observe(row);
        check(!result.triggered && result.fresh_position_codes==0,"older feedback sequences must not grow guard evidence");
        row.feedback_sequence=500+k;row.time_s=.01;
        check(!guard.observe(row).triggered,"out-of-order command times must not erase and regrow the active window");
        row=sample(50+k,.2*((k%2)?1:-1));row.feedback_sequence=0;
        check(!guard.observe(row).triggered,"a missing sequence must not reset a previously established feedback identity");
        row.feedback_sequence=500+k;row.command_time_s=row.time_s;row.feedback_time_s=row.time_s-.1;
        check(!guard.observe(row).triggered,"expired feedback must not become fresh oscillation evidence");
    }
    const auto after=guard.observe(sample(50,step));
    check(after.fresh_position_codes==before.fresh_position_codes && after.window_s>.15,
        "repeated/expired frames must preserve rather than reset established fresh evidence");

    NoiseEstimate larger;larger.encoder_step_rad=.01;guard.update_noise(larger);
    const auto unchanged=guard.observe(sample(51,step));
    check(unchanged.effective_encoder_step_rad<=step+1e-12,"a later larger resolution estimate cannot expand an established guard allowance");
    Requirements tight,loose;tight.peak_error_rad=1e-7;loose.peak_error_rad=.5;
    CommandConditionedMotionGuard a(tight),b(loose);
    check(a.observe(sample(0,0)).hysteresis_rad==b.observe(sample(0,0)).hysteresis_rad,
        "an unknown encoder must not use the final accuracy acceptance target as its safety motion scale");
    NoiseEstimate known;known.encoder_step_rad=step;
    CommandConditionedMotionGuard c(tight,known),d(loose,known);
    check(c.observe(sample(0,0)).hysteresis_rad==d.observe(sample(0,0)).hysteresis_rad,
        "known-resolution protection must also remain independent of the final accuracy target");
    known.encoder_step_rad=.0001;CommandConditionedMotionGuard drift({},known);bool drift_triggered=false;
    MotionGuardResult diagnostic;
    for(std::size_t k=0;k<500;++k){diagnostic=drift.observe(sample(k,.00045*(static_cast<int>(k%3)-1),.004*k*.005));
        drift_triggered=drift_triggered || diagnostic.triggered;}
    check(!drift_triggered && diagnostic.residual_peak_to_peak_rad>4*diagnostic.hysteresis_rad &&
        diagnostic.actual_peak_to_peak_rad<4*diagnostic.hysteresis_rad,
        "monotone reference drift must not inflate a sub-threshold actual motion into a sustained-oscillation amplitude");
}
void independent_input_checks() {
    NoiseEstimate noise;noise.encoder_step_rad=.0003814697265625;
    for(unsigned kind=0;kind<7;++kind) {
        CommandConditionedMotionGuard guard({},noise);bool triggered=false;MotionGuardResult last;
        for(std::size_t k=0;k<800;++k) {
            const double t=.005*k;
            auto row=sample(k,.009*std::sin(2*pi*25*(t-.02)));
            row.command_time_s=t;row.feedback_time_s=t-.015;
            row.independent_disturbance_known=kind!=4;
            row.independent_disturbance_nm=.06*std::sin(2*pi*25*t);
            if(kind==1) {
                row.actual=0;row.independent_disturbance_nm=0;
                const std::array<double,3> hz{13,21,29},input{.5,.3,.2},output{.7,.2,.1};
                for(std::size_t j=0;j<hz.size();++j){
                    row.independent_disturbance_nm+=.06*input[j]*std::sin(2*pi*hz[j]*t);
                    row.actual+=.009*output[j]*std::sin(2*pi*hz[j]*(t-.02)-.1*j);
                }
            }
            if(kind==2){row.independent_disturbance_nm=.06*std::sin(2*pi*2*t);
                row.actual+=.001*std::sin(2*pi*2*(t-.02));}
            if(kind==3)row.independent_disturbance_nm=.06;
            if(kind==5)row.independent_disturbance_nm=0;
            if(kind==6)row.independent_disturbance_known=k%10!=0;
            last=guard.observe(row);triggered=triggered || last.triggered;
        }
        check(triggered==(kind>=2),
            "known delayed single/three-tone forced motion is requested; low-frequency torque plus rapid extra motion, DC, zero or unknown input must retain protection");
        if(kind<2)check(last.independent_input_known && last.independent_input_reversals>0 &&
            last.requested_motion_reversals==last.independent_input_reversals,
            "the guard must record the actual known-input turn evidence rather than treating the fixed reference as the entire command");
        if(kind==3 || kind==5)check(last.independent_input_reversals==0,
            "a nonzero constant bias or zero independent torque does not request oscillation");
        if(kind==4 || kind==6)check(!last.independent_input_known && last.independent_input_reversals==0,
            "incomplete known-input evidence must not excuse actual motion");
    }
}
void production_probe_high_q_check() {
    const double inertia=.01,omega=2*pi*20,kp=inertia*omega*omega,kd=2*.02*inertia*omega,dt=.00025;
    MitFrequencyPlanRequest request;request.inertia_kg_m2=inertia;request.actual_kp=kp;request.actual_kd=kd;
    request.feedback_period_s=.005;request.feedback_age_s=.004;request.measurement_budget_s=100;request.per_controller_overhead_s=2.4;
    const auto plan=plan_mit_frequency_experiment(request);
    check(plan.eligible && plan.frequencies_hz==std::array<double,3>{5,10,20},"the stable high-Q regression must use an actual admitted production frequency plan");
    Requirements req;req.frequencies_hz=plan.frequencies_hz;req.probe_duration_s=plan.probe_duration_s;req.amplitude_rad=.006;
    IndependentDisturbanceDirectionPlan input;input.eligible=input.bidirectional=true;input.requested_torque_nm=.3;
    NoiseEstimate noise;noise.encoder_step_rad=.0003814697265625;
    CommandConditionedMotionGuard raw(req,noise),conditioned(req,noise),extra(req,noise),dc(req,noise);
    conditioned.set_independent_input_frequencies(plan.frequencies_hz);extra.set_independent_input_frequencies(plan.frequencies_hz);
    dc.set_independent_input_frequencies(plan.frequencies_hz);
    bool raw_triggered=false,conditioned_triggered=false,extra_triggered=false,dc_triggered=false,explained=false,unexplained=false;
    double q=0,v=0,max_q=0,explained_r_squared=0,condition=0;
    const auto acceleration=[&](double t,double x,double velocity){
        return (independent_disturbance_input(input,t-.25,req)-kp*x-kd*velocity)/inertia;
    };
    const auto steps=static_cast<unsigned>((req.probe_duration_s+.25+req.tail_s)/dt);
    for(unsigned k=0;k<steps;++k) {
        const double t=k*dt,a1=acceleration(t,q,v),x1=v;
        const double a2=acceleration(t+dt/2,q+dt*x1/2,v+dt*a1/2),x2=v+dt*a1/2;
        const double a3=acceleration(t+dt/2,q+dt*x2/2,v+dt*a2/2),x3=v+dt*a2/2;
        const double a4=acceleration(t+dt,q+dt*x3,v+dt*a3),x4=v+dt*a3;
        q+=dt*(x1+2*x2+2*x3+x4)/6;v+=dt*(a1+2*a2+2*a3+a4)/6;max_q=std::max(max_q,std::abs(q));
        if(k%20)continue;
        auto row=sample(k/20,q);row.command_time_s=row.feedback_time_s=t;
        row.independent_disturbance_known=true;row.independent_disturbance_nm=independent_disturbance_input(input,t-.25,req);
        raw_triggered=raw_triggered || raw.observe(row).triggered;
        const auto actual=conditioned.observe(row);conditioned_triggered=conditioned_triggered || actual.triggered;
        if(actual.input_tone_response_explained){explained=true;explained_r_squared=actual.input_tone_response_r_squared;condition=actual.input_tone_response_condition_number;}
        auto bad=row;if(t>1)bad.actual+=.007*std::sin(2*pi*35*t);
        const auto additional=extra.observe(bad);extra_triggered=extra_triggered || additional.triggered;
        unexplained=unexplained || additional.input_tone_unexplained_oscillation;
        auto constant=row;constant.independent_disturbance_nm=.06;dc_triggered=dc_triggered || dc.observe(constant).triggered;
        if(actual.triggered)throw std::runtime_error("stable production-probe forced response still triggered: "+actual.input_tone_explanation_reason+
            "; R_squared="+std::to_string(actual.input_tone_response_r_squared)+"; t="+std::to_string(t));
    }
    check(raw_triggered && !conditioned_triggered && explained && max_q<req.local_range_rad,
        "the real .4/.35/.25 probe, .7s fades and stable high-Q release require declared actual-plan tone conditioning, not a fabricated pass");
    check(extra_triggered && unexplained,"a resolved 35Hz oscillation beyond declared 5/10/20Hz input tones must retain the existing stop protection");
    check(dc_triggered,"declaring a tone plan without an actual alternating torque does not excuse DC-driven or unrelated oscillation");
    std::cout<<"production 5/10/20Hz probe stable high-Q release: explained R_squared="<<explained_r_squared
        <<", normalized condition="<<condition<<", raw maximum="<<max_q<<" rad; extra35Hz/DC remain guarded\n";
}
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> fields;std::stringstream stream(line);std::string field;
    while(std::getline(stream,field,','))fields.push_back(field);
    return fields;
}
void recorded_check(const std::string& filename) {
    std::ifstream input(filename);check(bool(input),"recorded raw response CSV cannot be opened");
    std::string line;std::getline(input,line);const auto header=split(line);
    const auto col=[&](const char* name){const auto it=std::find(header.begin(),header.end(),name);
        check(it!=header.end(),"recorded response lacks a required raw evidence column");return static_cast<std::size_t>(it-header.begin());};
    const auto jt=col("joint"),tm=col("time_s"),ac=col("actual"),rf=col("reference"),sq=col("feedback_sequence"),nm=col("new_measurement");
    const auto cmd=col("command_time_s"),feedback=col("feedback_time_s"),input_torque=col("independent_disturbance_nm");
    std::map<std::string,CommandConditionedMotionGuard> guards;std::map<std::string,MotionGuardResult> last;
    std::size_t count=0;
    while(std::getline(input,line)) {
        const auto v=split(line);check(v.size()==header.size(),"recorded raw CSV row dimensions changed");
        Sample row;row.time_s=std::stod(v[tm]);row.actual=std::stod(v[ac]);row.reference=std::stod(v[rf]);
        row.feedback_sequence=std::stoull(v[sq]);row.new_measurement=v[nm]=="1";
        row.command_time_s=std::stod(v[cmd]);row.feedback_time_s=std::stod(v[feedback]);
        row.independent_disturbance_known=true;row.independent_disturbance_nm=std::stod(v[input_torque]);
        const auto result=guards[v[jt]].observe(row);check(!result.triggered,"recorded sub-code chatter with monotone reference must not be classified as sustained oscillation");
        if(row.new_measurement)last[v[jt]]=result;
        ++count;
    }
    check(count>100 && !last.empty(),"real response replay must contain nonempty cycle evidence");
    bool learned=false;
    for(const auto& value:last)learned=learned || value.second.encoder_step_known;
    check(learned,"the real response must establish its observed encoder step during playback");
    std::cout<<filename<<": "<<count<<" raw cycle rows, "<<guards.size()<<" axes, zero false guard triggers\n";
}
}
int main(int argc,char** argv) {
    try {synthetic_checks();independent_input_checks();production_probe_high_q_check();for(int k=1;k<argc;++k)recorded_check(argv[k]);
        std::cout<<"online position-resolution guard: passed\n";return 0;}
    catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
