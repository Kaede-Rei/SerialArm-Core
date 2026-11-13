#include "serial_arm/dynamics/gain_measurement_center.hpp"
#include "serial_arm/dynamics/servo_gain_tuning.hpp"
#include <iostream>
#include <stdexcept>
using namespace serial_arm::servo_tuning;
void check(bool good,const char* reason){if(!good)throw std::runtime_error(reason);}
DemonstratedCenterRequest request(std::vector<double> fresh,std::vector<double> room) {
    DemonstratedCenterRequest req;req.fresh_position=std::move(fresh);req.desired_room=std::move(room);
    const auto n=req.fresh_position.size();req.safety_lower.assign(n,-3);req.safety_upper.assign(n,3);
    req.entry_tolerance.assign(n,.08);return req;
}

void route_simplification_test() {
    const double code=.0003814697265625;
    for(unsigned n:{1U,3U,7U}) {
        std::vector<std::vector<double>> dense;std::vector<std::size_t> source;
        for(unsigned k=0;k<=500;++k) {
            std::vector<double> q;
            for(unsigned i=0;i<n;++i)q.push_back(.2*i+(i%2?-1.:1.)*(1+.1*i)*k*.0001);
            dense.push_back(q);source.push_back(1000+k);
        }
        const auto exact=simplify_demonstrated_route(dense,source);
        check(exact.valid && exact.route.size()==2 && exact.route.front()==dense.front() && exact.route.back()==dense.back() &&
            exact.source_indices==std::vector<std::size_t>{1000,1500},
            "dense numerically collinear 1/3/7-axis poses must retain exact endpoints and source indices without per-node stops");
        for(const auto tolerance:exact.tolerance_rad)check(tolerance<1e-12,
            "unknown position resolution permits numerical collinearity only, not a physical angle tolerance");
        auto reversed=dense;auto reverse_source=source;std::reverse(reversed.begin(),reversed.end());std::reverse(reverse_source.begin(),reverse_source.end());
        const auto back=simplify_demonstrated_route(reversed,reverse_source,std::vector<double>(n,code));
        check(back.valid && back.route.size()==2 && back.route.front()==dense.back() && back.route.back()==dense.front() &&
            back.source_indices==std::vector<std::size_t>{1500,1000},
            "route simplification must preserve full-pose reversed direction and original source identities");
        auto folded=dense;
        for(std::size_t k=0;k<folded.size();++k)for(unsigned i=0;i<n;++i) {
            const double s=k<=200?k*.0001:k<=300?.02-(k-200)*.0001:.01+(k-300)*.0002;
            folded[k][i]=.2*i+(i%2?-1.:1.)*(1+.1*i)*s;
        }
        const auto turns=simplify_demonstrated_route(folded,source,std::vector<double>(n,code));
        check(turns.valid && std::find(turns.source_indices.begin(),turns.source_indices.end(),1200)!=turns.source_indices.end() &&
            std::find(turns.source_indices.begin(),turns.source_indices.end(),1300)!=turns.source_indices.end(),
            "a genuine collinear out-and-back turn must not be erased by a spatial-only RDP chord");
        PathProgress progress;auto actual=exact.route.front();std::vector<double> velocity(n,0);unsigned departures=0;bool moving=false,completed=false;
        for(unsigned k=0;k<3000 && !completed;++k) {
            const auto next=advance_demonstrated_path(exact.route,progress,actual,std::vector<double>(n,.01),
                std::vector<double>(n,.05),std::vector<double>(n,.2),.005);
            check(next.valid,"the simplified route must retain the original hard lead and valid common-scalar dynamics");
            if(next.progress.speed>0 && !moving)++departures;
            moving=next.progress.speed>0;
            for(unsigned i=0;i<n;++i)check(std::abs(next.velocity[i]-velocity[i])<=.2*.005+1e-9 &&
                std::abs(next.position[i]-actual[i])<=.01+1e-12,
                "removing redundant nodes must preserve actual reference acceleration and hard following lead");
            progress=next.progress;actual=next.position;velocity=next.velocity;completed=next.complete;
        }
        check(completed && actual==dense.back() && departures==1,
            "500 dense collinear samples must execute as one accelerate/brake move, not 500 restarts");
        check(demonstrated_path_transition_seconds(exact.route,std::vector<double>(n,.05),std::vector<double>(n,.2))<
            demonstrated_path_transition_seconds(dense,std::vector<double>(n,.05),std::vector<double>(n,.2)),
            "the migration estimate must charge the actually simplified move rather than phantom per-code stops");
    }
    std::vector<std::vector<double>> sub_code;std::vector<std::size_t> source;
    for(unsigned k=0;k<=100;++k){sub_code.push_back({.001*k,.002*k+.2*code*std::sin(2*3.14159265358979323846*k/10)});source.push_back(k);}
    const auto measured=simplify_demonstrated_route(sub_code,source,{code,code});
    const auto unknown=simplify_demonstrated_route(sub_code,source,{0,0});
    check(measured.valid && measured.route.size()==2 && unknown.valid && unknown.route.size()>2,
        "sub-code geometry may be merged only for axes whose position resolution has actually been observed");
    for(const auto tolerance:measured.tolerance_rad)check(tolerance==code/2,
        "the geometric allowance must be exactly half the observed per-axis position step");
    const std::vector<std::vector<double>> corner{{0,0},{.01,0},{.02,0},{.02,.01},{.02,.02}};
    const auto geometric=simplify_demonstrated_route(corner,{0,1,2,3,4},{code,code});
    check(geometric.valid && geometric.route==std::vector<std::vector<double>>{corner.front(),corner[2],corner.back()} &&
        geometric.source_indices==std::vector<std::size_t>{0,2,4},
        "a real multi-axis corner must retain its complete original pose and direction change");
    std::vector<std::vector<double>> path;
    for(unsigned k=0;k<=500;++k)path.push_back({.0001*k});
    auto req=request({-.0001},{.015});req.observed_position_step_rad={code};
    const auto entry=plan_demonstrated_measurement_center(path,req);
    check(entry.valid && entry.center==path[entry.target_source_index] && entry.route.front()==req.fresh_position &&
        entry.route.back()==entry.center && entry.route_original_nodes>entry.route.size() &&
        entry.demonstrated_route_original_nodes>entry.demonstrated_route.size(),
        "entry simplification must keep the exact selected center and fresh bridge while recording original node counts");
    check(entry.source_indices.front()==std::numeric_limits<std::size_t>::max() && entry.route.size()==3 && entry.route[1]==path.front(),
        "an accepted fresh position outside the demonstrated envelope must retain its distinct original entry-bridge junction");
    const auto returning=plan_demonstrated_measurement_return(entry,entry.center,path.front(),0,req.entry_tolerance,{code});
    check(returning.valid && returning.route.front()==entry.center && returning.route.back()==path.front() &&
        returning.center==path.front() && returning.route.size()==2 && returning.route_simplification_tolerance_rad==std::vector<double>{code/2},
        "return simplification must retain the original endpoint and its own measured-resolution metadata");
    check(!simplify_demonstrated_route(corner,{0,1,2,3,4},{code,-code}).valid,
        "invalid resolution cannot authorize a geometric shortcut");
    check(!plan_demonstrated_measurement_return(entry,entry.center,path.front(),0,req.entry_tolerance,{-code}).valid,
        "return must reject invalid resolution even after copying an otherwise valid entry plan");
}

void recorded_endpoint_reference_test() {
    // Counterfactual reference-only replay from task 1794291242687. These are
    // the captured control-cycle times and raw positions, not altered feedback
    // or a claim that the physical tracking error meets its separate target.
    const std::uint64_t time_ns[]{
        4750490,9752693,14887877,19955384,25117876,30130765,35142467,40278838,45602807,50372295,55418429,60455065,
        65554629,70585328,75673019,80937622,85910142,90896910,96516523,101109099,106136236,111227489,116249877,121919357,
        126440694,131574690,136550772,142089647,146844887,151905269,156844544,161973792,167041298,172140862,177512324,182220070,
        187470425,192437008,197847652,202577958,207577786,212644105,217743669,222847983,227914302,232971122,238311713,243129881,
        248171267,253238773,258416700,263388033,268533903,273836500,278960998,283706739,288919099,293986605,298954376,304057502,
        309115510,314432355,319371630,324340588,329435403,334571774,339446934,344698476,349673371,354664888,359768014,364848581,
        370110809,374961034,380160334,385218342,390311969,395362853,400378117,405439687,410585556,415616256,420707508,425728709,
        430749910,435965832,440988220,446140026,451062679,456176491,461211939,466312691,471327955,476407334,481462967,486667016,
        491538613,496686858,501849350,506912107,511908374,517000814,522170431,527099020,532129719,537413319,542387026,547459282,
        552835495,557848386,562901647,567966782,573107905,578256152,583041077,588189324,593113166,598202047,603797916,608962786,
        613391513,618586066,623521781,629961839,633850333,638991456,643909362,649042174,654172611,659292362,664206706,669631600,
        674390404,679437728,684500488,690112979,694635505,699681642,706660747,709993572,714999339,720040727,725316018,730207803,
        735218320,740391501,745378271,750512270,755912231,760701905,765720734,770854733,775903244,780986188,786056072,791039281,
        796118663,801266910,806411595,811282007,816331706,821973881,826663820,831626844,836672981,841726242,846932668,851990678,
        857161485,
    };
    const std::vector<double> origin{-.00972747802734375,.00667572021484375,.02117156982421875,
        .00743865966796875,-.00133514404296875,.00782012939453125};
    const std::vector<double> target{-.004386901855468748,.005912780761718749,.016593933105468743,
        .005531311035156248,-.0013351440429687498,.00438690185546875};
    const double code=.0003814697265625;
    const std::vector<double> lead(6,.01),max_velocity(6,.05),max_acceleration(6,.2);
    std::vector<double> uncertainty;
    for(double hard_lead:lead)uncertainty.push_back(measurement_path_feedback_reserve(hard_lead,code,0).position_uncertainty);
    const std::vector<std::vector<double>> route{origin,target};
    const double geometric=demonstrated_path_transition_seconds(route,max_velocity,max_acceleration);
    const double allocated=std::max(.8,geometric+.7+.4);
    PathProgress progress;std::vector<double> previous=origin,previous_velocity(6,0),actual=origin;
    bool reached=false;double reached_at=-1;
    for(std::size_t k=1;k<sizeof(time_ns)/sizeof(time_ns[0]);++k) {
        // Only this recorded joint changed a position code during the failed
        // transition. All other axes kept their original received position.
        if(k==34 || k==40 || k==43 || k==168)actual[3]=.00705718994140625;
        if(k==35 || k==42 || k==167)actual[3]=.00743865966796875;
        const double dt=static_cast<double>(time_ns[k]-time_ns[k-1])*1e-9;
        const auto step=advance_demonstrated_path(route,progress,actual,lead,max_velocity,max_acceleration,dt,uncertainty);
        check(step.valid,"recorded legal quantized endpoint feedback must admit a bounded finite bridge");
        const double alpha=(step.position[0]-origin[0])/(target[0]-origin[0]);
        for(std::size_t i=0;i<6;++i) {
            check(std::abs(step.position[i]-(origin[i]+alpha*(target[i]-origin[i])))<1e-12,
                "the endpoint bridge preserves a common whole-pose segment instead of moving joint goals with feedback");
            check(std::abs(step.position[i]-actual[i])<=lead[i]+1e-12,
                "recorded endpoint replay retains the original hard reference-error cap against unchanged raw feedback");
            check(std::abs(step.velocity[i])<=max_velocity[i]+1e-12 &&
                std::abs(step.velocity[i]-previous_velocity[i])<=max_acceleration[i]*dt+1e-9,
                "finite endpoint arrival must retain actual reference velocity and acceleration bounds");
            check((target[i]-origin[i])*(step.position[i]-previous[i])>=-1e-15,
                "a fixed endpoint bridge must not develop the old capture-and-reverse reference tail oscillation");
        }
        const double elapsed=static_cast<double>(time_ns[k])*1e-9;
        if(step.complete && !reached){reached=true;reached_at=elapsed;}
        if(reached)check(step.complete && step.position==target &&
            std::all_of(step.velocity.begin(),step.velocity.end(),[](double v){return v==0;}),
            "after finite arrival, every executed endpoint reference must remain exactly at target with zero reference velocity");
        progress=step.progress;previous=step.position;previous_velocity=step.velocity;
    }
    check(reached && reached_at+.15<static_cast<double>(time_ns[sizeof(time_ns)/sizeof(time_ns[0])-1])*1e-9 && reached_at+.15<allocated,
        "all six recorded endpoint references must reach exact rest and execute a real dwell within the explicit transition allocation");
    auto one_code_back=actual;one_code_back[0]-=code;
    const auto dwell_code=advance_demonstrated_path(route,progress,one_code_back,lead,max_velocity,max_acceleration,.005,uncertainty);
    check(dwell_code.valid && dwell_code.complete && dwell_code.position==target &&
          std::all_of(dwell_code.velocity.begin(),dwell_code.velocity.end(),[](double value){return value==0;}),
        "a legal one-code raw-feedback return during endpoint dwell retains exact zero-velocity reference without enlarging its lead");
    auto excessive_return=target;excessive_return[0]-=lead[0]+code;
    const auto rejected_dwell=advance_demonstrated_path(route,progress,excessive_return,lead,max_velocity,max_acceleration,.005,uncertainty);
    check(!rejected_dwell.valid && rejected_dwell.reason=="path_reference_outside_feedback_lead",
        "completed path state cannot hide a real raw-feedback retreat beyond the unchanged hard lead during zero-velocity dwell");
    const std::vector<std::vector<double>> stationary_node{origin,origin,target};
    const auto accepted_stationary=advance_demonstrated_path(stationary_node,{},one_code_back,lead,max_velocity,max_acceleration,.005,uncertainty);
    check(accepted_stationary.valid && !accepted_stationary.complete && accepted_stationary.position==origin &&
          std::all_of(accepted_stationary.velocity.begin(),accepted_stationary.velocity.end(),[](double value){return value==0;}),
        "an actually stationary path node retains legal quantized feedback and a zero-velocity reference");
    excessive_return=origin;excessive_return[1]+=lead[1]+code;
    const auto rejected_stationary=advance_demonstrated_path(stationary_node,{},excessive_return,lead,max_velocity,max_acceleration,.005,uncertainty);
    check(!rejected_stationary.valid && rejected_stationary.reason=="path_reference_outside_feedback_lead",
        "a zero-length demonstrated node must not bypass the original hard-feedback lead on any axis");
    check(std::abs(actual[0]-target[0])>.1*3.14159265358979323846/180 &&
          std::abs(actual[0]-target[0])<.08,
        "endpoint reference completion is independent of the unchanged guide tolerance and the separate 0.1-degree measured tracking acceptance");
    // Existing alignment can admit a bridge longer than the available feedback
    // lead. A plant which does not follow must stop, rather than falsely finish
    // or push the reference through its physical lead boundary.
    auto distant=target;distant[0]=origin[0]+.06;
    const std::vector<std::vector<double>> stalled_route{origin,distant};
    progress={};previous_velocity.assign(6,0);bool moved=false,stopped=false;
    for(unsigned k=0;k<2000;++k) {
        const auto step=advance_demonstrated_path(stalled_route,progress,origin,lead,max_velocity,max_acceleration,.005,uncertainty);
        check(step.valid && !step.complete,"stalled endpoint feedback must remain an incomplete bounded transition");
        for(std::size_t i=0;i<6;++i) {
            check(std::abs(step.position[i]-origin[i])<=lead[i]+1e-12,
                "feedback waiting must not increase the endpoint hard lead");
            check(std::abs(step.velocity[i]-previous_velocity[i])<=max_acceleration[i]*.005+1e-9,
                "a feedback-bounded endpoint pause must brake its actual commanded velocity");
        }
        moved=moved || step.progress.speed>0;stopped=stopped || (moved && step.progress.speed==0);
        progress=step.progress;previous_velocity=step.velocity;
    }
    check(moved && stopped,"a real lack of endpoint following must end in a stopped reference, not a fabricated arrival");
}

int main() {
    try {
        route_simplification_test();
        recorded_endpoint_reference_test();
        const double encoder_code=.0003814697265625;
        ObservedPositionStep stationary;
        for(std::uint64_t sequence=1;sequence<=100;++sequence)
            check(stationary.observe(.7,sequence),"constant positions are valid fresh feedback, while their encoder step remains unknown");
        check(stationary.step()==0 && stationary.fresh_samples()==100,
            "a stationary constant trace must not fabricate an identified encoder resolution");
        check(!stationary.observe(.7+encoder_code,100) && !stationary.observe(.7+encoder_code,99) &&
              !stationary.observe(.7+encoder_code,101,false) && !stationary.observe(std::numeric_limits<double>::quiet_NaN(),101),
            "duplicate, reversed, stale and nonfinite feedback must not enter the streamed position-step estimate");
        check(stationary.step()==0 && stationary.last_sequence()==100 && stationary.fresh_samples()==100,
            "rejected feedback must not invent position evidence or consume the next valid sequence");
        check(stationary.observe(.7+encoder_code,101) && stationary.step()==encoder_code,
            "the first real fresh code transition must establish the observable minimum position step");
        for(unsigned order:{0U,2U,1U}) {
            ObservedPositionStep stream;std::vector<double> positions;
            for(double code_count:{0.,4.,1.,3.,2.,1.,-.5,8.}) {
                const double position=std::vector<double>{-.83,.7,1.4}[order]+code_count*encoder_code*(order+1);
                positions.push_back(position);check(stream.observe(position,positions.size()),"each joint's own fresh sequence must be accepted");
                check(stream.step()==observed_encoder_step(positions),
                    "streamed nearest-neighbor resolution must agree with batch evidence for every axis/order/nonzero offset");
            }
        }
        ObservedPositionStep tiny;std::vector<double> near_values;
        for(double position:{0.,2e-9,1e-9}) {
            near_values.push_back(position);tiny.observe(position,near_values.size());
            check(tiny.step()==observed_encoder_step(near_values),
                "inserting a sub-threshold neighbor must remove an obsolete gap exactly as batch position evidence does");
        }
        const auto unidentified=measurement_path_feedback_reserve(.01,0,0);
        check(unidentified.valid && !unidentified.encoder_step_known && unidentified.position_uncertainty==.0025 &&
              unidentified.reason=="unidentified_position_step_internal_guard_buffer",
            "unknown resolution uses one quarter of the existing hard lead as a guard buffer, not a claimed encoder quantum");
        const auto identified=measurement_path_feedback_reserve(.01,encoder_code,0);
        check(identified.valid && identified.encoder_step_known && identified.position_uncertainty==3*encoder_code,
            "observable code steps require three-code planning reserve inside the unchanged hard lead");
        check(measurement_path_feedback_reserve(.01,encoder_code,.0003).position_uncertainty==6*.0003 &&
              !measurement_path_feedback_reserve(.001,encoder_code,0).valid &&
              !measurement_path_feedback_reserve(.01,-encoder_code,0).valid,
            "stationary noise and resolution remain actual input constraints, never reasons to enlarge a hard lead");
        std::vector<std::vector<double>> path;
        for(int k=0;k<=10;++k)path.push_back({.4+.01*k,1.2-.01*k,.7});
        auto req=request({.415,1.185,.7},{.02,.02,.01});req.priority_axes={true,true,false};
        const auto plan=plan_demonstrated_measurement_center(path,req);
        check(plan.valid && plan.route.front()==req.fresh_position && plan.center==path[2],
            "choose the earliest sufficient complete demonstrated pose, starting with fresh feedback");
        check(plan.route.size()>=2 && plan.route[1][0]>=req.fresh_position[0] && plan.route[1][1]<=req.fresh_position[1],
            "entry must skip the old endpoint instead of pulling an already inward fresh position back toward it");
        check(plan.insufficient_axes==std::vector<std::size_t>{2} && !plan.room_satisfied[2],
            "an axis with no demonstrated travel retains missing room rather than borrowing a URDF interval");
        for(std::size_t k=1;k<plan.route.size();++k)check(std::abs(plan.route[k][0]+plan.route[k][1]-1.6)<1e-12 && plan.route[k][2]==.7,
            "correlated demonstrated coordinates must not be replaced by independently selected axis extrema");
        auto translated=path;for(auto& q:translated)for(auto& x:q)x+=.9;
        auto shifted=req;for(auto& x:shifted.fresh_position)x+=.9;
        const auto moved=plan_demonstrated_measurement_center(translated,shifted);
        check(moved.valid && moved.target_source_index==plan.target_source_index && moved.route.size()==plan.route.size(),
            "hard-stop direction must follow demonstrated geometry, not an angle equal to zero");
        auto reversed=path;std::reverse(reversed.begin(),reversed.end());auto backwards=req;backwards.reverse=true;
        const auto back=plan_demonstrated_measurement_center(reversed,backwards);
        check(back.valid && back.center==plan.center && back.target_source_index==path.size()-1-plan.target_source_index,
            "the selected actual endpoint determines continuous recorded direction");
        const std::vector<std::vector<double>> wiggle{{.4},{.414},{.409},{.435},{.47},{.4}};
        const auto joined=plan_demonstrated_measurement_center(wiggle,request({.412},{.02}));
        check(joined.valid && joined.center==wiggle[3] && joined.entry_source_index==2,
            "a small endpoint wiggle must be skipped before entry without waiting for a later path return");
        for(const auto& q:joined.route)check(q[0]>=.412,
            "joining a demonstrated interior path must not first revisit a mechanical endpoint behind fresh feedback");
        const auto returning=plan_demonstrated_measurement_return(joined,{.425},wiggle.front(),0,{.08});
        check(returning.valid && returning.route.front()==std::vector<double>{.425} && returning.route.back()==wiggle.front() &&
              returning.center==wiggle.front() && returning.reverse!=joined.reverse,
            "return starts at fresh and ends at the original demonstrated endpoint with correct direction metadata");
        check(returning.route.size()>2 && returning.route[1][0]<=.425 &&
              std::find(returning.route.begin()+1,returning.route.end(),wiggle[1])!=returning.route.end() &&
              std::find(returning.route.begin()+1,returning.route.end(),wiggle[2])!=returning.route.end(),
            "return skips the old nominal target and follows the full demonstrated endpoint wiggle previously skipped on entry");
        check(std::none_of(returning.route.begin()+1,returning.route.end(),[](const auto& q){return q==std::vector<double>{.412};}),
            "original fresh feedback is not a demonstrated return waypoint");
        check(!plan_demonstrated_measurement_return(joined,{.425},{.401},0,{.08}).valid,
            "return must not silently replace the original endpoint or reconstruct it from axis extrema");
        check(!plan_demonstrated_measurement_return(joined,{.6},wiggle.front(),0,{.08}).valid,
            "return projection retains the same existing alignment allowance");
        auto offset=request({.4-.079,1.2,.7},{.02,.02,.01});offset.priority_axes={true,false,false};
        check(plan_demonstrated_measurement_center(path,offset).valid,
            "a fresh legal position within the existing 0.08 alignment allowance must remain a supported start");
        offset.fresh_position[0]=.4-.081;
        check(!plan_demonstrated_measurement_center(path,offset).valid,
            "the planner cannot invent a larger endpoint alignment allowance");
        auto safety=req;safety.safety_lower[0]=.416;
        check(!plan_demonstrated_measurement_center(path,safety).valid,
            "fresh outside explicit Safety bounds must be rejected rather than repaired by reference clamping");
        std::vector<std::vector<double>> corner{{.4,.5},{.45,.5},{.5,.5},{.5,.55},{.5,.6}};
        const auto partial=plan_demonstrated_measurement_center(corner,request({.4,.5},{.02,.02}));
        check(partial.valid && !partial.insufficient_axes.empty() && partial.reason=="partial_measurement_room_on_demonstrated_path",
            "no common two-axis interior pose must remain partial instead of combining independent midpoints");
        // Strict common-progress motion: stop at a real corner, then change
        // direction without a joint-velocity jump or an off-path pose.
        const std::vector<std::vector<double>> route{{.4,.5},{.42,.5},{.42,.52}};
        PathProgress progress;std::vector<double> actual=route.front(),last_velocity(2,0);
        bool complete=false;double peak_speed=0;
        for(int k=0;k<20000 && !complete;++k) {
            const auto step=advance_demonstrated_path(route,progress,actual,{.02,.02},{.08,.08},{.2,.2},.002);
            check(step.valid,"a well-following plant must advance with a legal common scalar path state");
            check(step.position[0]==.42 || step.position[1]==.5,"every commanded reference must remain on one actual complete path segment");
            for(unsigned i=0;i<2;++i){check(std::abs(step.velocity[i]-last_velocity[i])<=.2*.002+1e-9,"true reference acceleration must remain bounded across corners");peak_speed=std::max(peak_speed,std::abs(step.velocity[i]));}
            progress=step.progress;actual=step.position;last_velocity=step.velocity;complete=step.complete;
        }
        check(complete && actual==route.back() && peak_speed>0,"actual path progress must reach its requested interior point");
        check(demonstrated_path_transition_seconds(route,{.08,.08},{.2,.2})>1,
            "migration cost must include acceleration and stopping at polyline corners");
        // If actual feedback stops following, brake while preserving the lead;
        // do not zero velocity as soon as a positional clamp is encountered.
        progress={};double previous_speed=0;bool saw_motion=false,saw_deceleration=false,saw_stop=false;
        const std::vector<std::vector<double>> straight{{.4},{.5}};
        for(int k=0;k<4000;++k) {
            const auto step=advance_demonstrated_path(straight,progress,{.4},{.02},{.1},{.25},.002);
            check(step.valid && step.position[0]<=.42+1e-12,"a stalled actuator must not consume more than the bounded reference lead");
            const double v=step.velocity[0];
            check(std::abs(v-previous_speed)<=.25*.002+1e-9,"feedback lead pauses must decelerate the actual commanded reference velocity");
            saw_motion=saw_motion || v>.001;saw_deceleration=saw_deceleration || (previous_speed>v && v>0);
            saw_stop=saw_stop || (saw_motion && v==0);previous_speed=v;progress=step.progress;
        }
        check(saw_motion && saw_deceleration && saw_stop,"stalled feedback must lead to an actual acceleration-limited stop");
        // Real calling-chain gap: a stationary trace initially identifies no
        // step, then motion exposes the first code and its immediate return.
        // Update the streamed evidence before planning the next reference.
        progress={};ObservedPositionStep discovered;bool discovered_step=false;previous_speed=0;
        for(int k=0;k<2400;++k) {
            const double received=.4+(k==1200?encoder_code:0);
            check(discovered.observe(received,k+1),"each new raw feedback packet must update the streaming step tracker");
            const auto reserve=measurement_path_feedback_reserve(.01,discovered.step(),0);
            check(reserve.valid,"a newly observed useful code must yield a legal internal stopping reserve");
            check(k>=1200 || !reserve.encoder_step_known,"zero motion cannot be reported as successful encoder identification");
            discovered_step=discovered_step || reserve.encoder_step_known;
            const auto step=advance_demonstrated_path(straight,progress,{received},{.01},{.05},{.2},.005,{reserve.position_uncertainty});
            check(step.valid && std::abs(step.position[0]-received)<=.01+1e-12,
                "unknown stationary feedback followed by a code/return must never consume the original hard lead");
            check(std::abs(step.velocity[0]-previous_speed)<=.2*.005+1e-9,
                "learning the first position step must preserve actual reference acceleration and braking");
            progress=step.progress;previous_speed=step.velocity[0];
        }
        check(discovered_step && discovered.step()==encoder_code,"motion must actually change the resolution evidence used by planning");
        const auto recoil=advance_demonstrated_path(straight,progress,{.38},{.01},{.05},{.2},.005,{identified.position_uncertainty});
        check(!recoil.valid && recoil.reason=="path_reference_outside_feedback_lead",
            "a real large reverse movement remains a hard-lead violation rather than being relabeled as quantization noise");
        PathProgress moving{0,.005,.05};
        check(measurement_path_lead_change_allowed(moving,{.01,.02},{.01,.02}) &&
              !measurement_path_lead_change_allowed(moving,{.01,.02},{.009,.02}) &&
              measurement_path_lead_change_allowed(PathProgress{0,.005,0},{.01,.02},{.009,.02}),
            "same-lead gain steps remain available, while any tighter lead is deferred until the shared path reference stops");
        check(advance_demonstrated_path(straight,moving,{.4},{.01},{.05},{.25},.002,{0}).valid &&
              !advance_demonstrated_path(straight,moving,{.4},{.009},{.05},{.25},.002,{0}).valid,
            "position error alone cannot authorize a gain step that removes already committed braking distance");
        // Counterfactual reference-generator replay of the actual last feedback
        // pair from task 1794288727413; values remain unmodified. This checks
        // the unsafe resume decision, not an unexecuted physical-arm outcome.
        const std::vector<std::vector<double>> recorded_route{{.01049041748046875},{.12519706844636067}};
        const PathProgress parked{0,.022015296386718751-.01049041748046875,0};
        const auto old_resume=advance_demonstrated_path(recorded_route,parked,{.01239776611328125},{.01},{.05},{.2},.005,{1e-6});
        check(old_resume.valid && old_resume.velocity[0]>0 &&
              !advance_demonstrated_path(recorded_route,old_resume.progress,{.01201629638671875},{.01},{.05},{.2},.005,{1e-6}).valid,
            "the recorded forward-code/return pair must reproduce the former unsafe resume with a one-microradian reserve");
        const auto guarded=advance_demonstrated_path(recorded_route,parked,{.01239776611328125},{.01},{.05},{.2},.005,{identified.position_uncertainty});
        check(guarded.valid && guarded.velocity[0]==0 && guarded.position[0]==.022015296386718751 &&
              advance_demonstrated_path(recorded_route,guarded.progress,{.01201629638671875},{.01},{.05},{.2},.005,{identified.position_uncertainty}).valid,
            "the same unmodified recorded feedback must retain a stopped reference when its observed code is reserved before the resume decision");
        // Raw encoder feedback can alternate by a code near the stopped lead.
        // Planning reserves three codes, while the hard check still uses the
        // same original lead against the unmodified received position.
        progress={};const double code=.0003814697265625;previous_speed=0;saw_stop=false;
        for(int k=0;k<4000;++k) {
            const double received=.4+(k%2?-code:0);
            const auto step=advance_demonstrated_path(straight,progress,{received},{.02},{.1},{.25},.002,{3*code});
            check(step.valid && std::abs(step.position[0]-received)<=.02+1e-12,
                "one-code feedback retreats must retain the original hard lead using the preplanned uncertainty reserve");
            check(std::abs(step.velocity[0]-previous_speed)<=.25*.002+1e-9,
                "encoder code changes must not create an instantaneous reference-velocity stop");
            previous_speed=step.velocity[0];progress=step.progress;saw_stop=saw_stop || (k>1000 && previous_speed==0);
        }
        check(saw_stop,"quantized stalled feedback must still reach a real bounded stop");
        auto outside=progress;outside.distance=.03;outside.speed=0;
        check(!advance_demonstrated_path(straight,outside,{.4},{.02},{.1},{.25},.002,{3*code}).valid,
            "position uncertainty must never excuse a reference already beyond the original hard feedback lead");
        check(!advance_demonstrated_path(straight,{}, {.4},{.02},{.1},{.25},.002,{.02}).valid,
            "uncertainty consuming the complete lead is insufficient input resolution, not a successful motion plan");
        // A lower mechanical stop and static friction are physical plant
        // constraints. One-way path migration may build motion before any
        // bidirectional response claim; bounded P steps here emulate the
        // runtime's separate effort calibration, while D remains unchanged.
        progress={};double q=.4,v=0,kp=2,kd=.2;bool left_stop=false;unsigned increases=0;
        for(int k=0;k<30000;++k) {
            const double received=.4+std::round((q-.4)/code)*code+(k%173==172?-code:0);
            const auto step=advance_demonstrated_path(straight,progress,{received},{.01},{.05},{.15},.002,{3*code});
            check(step.valid,"the lower-stop friction plant must retain legal feedback-bounded scalar progress");
            check(std::abs(step.position[0]-received)<=.01+1e-12,
                "friction release with quantized raw feedback must retain the same physical reference lead");
            check(step.position[0]>=.4 && step.position[0]<=.5,"one-way release must never command through the mechanical endpoint");
            const double applied=kp*(step.position[0]-received)+kd*(step.velocity[0]-v);
            double torque=applied;
            if(std::abs(v)<1e-4 && std::abs(torque)<.08)torque=0;
            else torque-=.06*(v!=0?(v>0?1:-1):(torque>0?1:-1));
            const double acceleration=(torque-.35*v)/.03;
            v+=acceleration*.002;q+=v*.002;
            if(q<.4){q=.4;v=std::max(0.0,v);}
            if(k%500==499 && q<.4005 && increases<12){kp*=1.4;++increases;}
            left_stop=left_stop || q>.42;progress=step.progress;
            if(step.complete && std::abs(q-.5)<.001)break;
        }
        check(left_stop && q>.49 && increases>0 && kd==.2,
            "bounded one-way motion can leave a nonzero hard stop after static-friction P calibration without first demanding bidirectional evidence");
        std::cout<<"demonstrated measurement center and common path progress passed: original alignment tolerance, inward entry/return, correlated/reversed/translated poses, streamed fresh position resolution, unknown-step guard, recorded code-return counterfactual, finite six-axis endpoint arrival/dwell, stopped lead handoff, bounded velocity/acceleration and lower-stop friction release\n";
        return 0;
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
