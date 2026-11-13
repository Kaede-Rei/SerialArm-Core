// Read-only replay of archived online gain decisions against current code.
// This is a shadow decision check: it never invents feedback for proposed gains.
#include "serial_arm/dynamics/gain_online_trajectory_tuning.hpp"
#include "serial_arm/dynamics/gain_candidate_admission.hpp"
#include "serial_arm/dynamics/calibration_replay_supervisor.hpp"
#include <yaml-cpp/yaml.h>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>

using namespace serial_arm::servo_tuning;
namespace fs=std::filesystem;
namespace {
std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> out;std::string field;std::istringstream in(line);
    while(std::getline(in,field,','))out.push_back(field);
    if(!line.empty() && line.back()==',')out.emplace_back();
    return out;
}
std::vector<double> numbers(const YAML::Node& node,std::size_t n,const char* name) {
    if(!node || !node.IsSequence() || node.size()!=n)throw std::runtime_error(std::string("missing mapped ")+name);
    std::vector<double> result;for(const auto& x:node)result.push_back(x.as<double>());return result;
}
YAML::Node vector_node(const std::vector<double>& values) {YAML::Node out(YAML::NodeType::Sequence);for(double x:values)out.push_back(x);return out;}
YAML::Node gains_node(const Gains& gains) {YAML::Node out;out["kp"]=vector_node(gains.kp);out["kd"]=vector_node(gains.kd);return out;}
using Rows=std::vector<std::vector<Sample>>;
Rows load_record(const fs::path& path,const std::vector<std::string>& joints) {
    std::ifstream input(path);if(!input)throw std::runtime_error("cannot read record: "+path.string());
    std::string line;std::getline(input,line);const auto header=split(line);std::map<std::string,std::size_t> columns;
    for(std::size_t i=0;i<header.size();++i)columns[header[i]]=i;
    std::map<std::string,std::size_t> axes;for(std::size_t i=0;i<joints.size();++i)axes[joints[i]]=i;
    for(const auto* required:{"time_s","joint","reference","reference_velocity","actual","velocity","new_measurement","independent_disturbance_nm","feedforward_nm","actual_kp","actual_kd","command_time_s","feedback_time_s"})
        if(!columns.count(required))throw std::runtime_error(std::string("record lacks ")+required);
    Rows rows(joints.size());
    while(std::getline(input,line)) {
        const auto fields=split(line);if(fields.size()!=header.size())throw std::runtime_error("CSV row width changed");
        const auto axis=axes.find(fields[columns.at("joint")]);if(axis==axes.end())throw std::runtime_error("CSV joint not mapped by task");
        const auto get=[&](const std::string& key,double fallback=0) {const auto c=columns.find(key);return c==columns.end()?fallback:std::stod(fields[c->second]);};
        Sample s;s.time_s=get("time_s");s.reference=get("reference");s.reference_velocity=get("reference_velocity");
        s.actual=get("actual");s.velocity=get("velocity");s.residual=get("residual");s.residual_valid=get("residual_valid")!=0;
        s.limited=get("limited")!=0;s.new_measurement=get("new_measurement")!=0;
        s.applied_feedforward_nm=get("feedforward_nm");s.feedforward_known=true;
        s.independent_disturbance_nm=get("independent_disturbance_nm");s.independent_disturbance_known=true;
        s.command_time_s=get("command_time_s");s.feedback_time_s=get("feedback_time_s");
        const auto sequence=columns.find("feedback_sequence");s.feedback_sequence=sequence==columns.end()?0:std::stoull(fields[sequence->second]);
        s.actual_kp=get("actual_kp");s.actual_kd=get("actual_kd");s.actual_gains_known=get("actual_gains_known",1)!=0;
        s.reference_sample_time_s=get("reference_sample_time_s");s.reference_sample_time_known=get("reference_sample_time_known")!=0;
        s.measured_torque_nm=get("measured_torque_nm",std::numeric_limits<double>::quiet_NaN());
        s.planned_independent_disturbance_nm=get("planned_independent_disturbance_nm");s.independent_input_planned_known=get("independent_input_planned_known")!=0;
        rows[axis->second].push_back(s);
    }
    for(const auto& axis:rows)if(axis.empty())throw std::runtime_error("record missing mapped axis");
    return rows;
}
const Sample& at(const std::vector<Sample>& rows,double time) {
    auto it=std::lower_bound(rows.begin(),rows.end(),time,[](const Sample& row,double t){return row.time_s<t;});
    return it==rows.end()?rows.back():*it;
}
double ref_at(const std::vector<Sample>& rows,double time,bool velocity) {
    const auto& hi=at(rows,time);const auto index=static_cast<std::size_t>(&hi-rows.data());
    if(!index || hi.time_s<=time)return velocity?hi.reference_velocity:hi.reference;
    const auto& lo=rows[index-1];const double a=std::clamp((time-lo.time_s)/(hi.time_s-lo.time_s),0.0,1.0);
    return (1-a)*(velocity?lo.reference_velocity:lo.reference)+a*(velocity?hi.reference_velocity:hi.reference);
}
YAML::Node model_node(const OnlineTrajectoryAxisModel& m) {
    YAML::Node out;out["valid"]=m.valid;out["reason"]=m.reason;out["fresh_samples"]=m.samples;
    out["model_dt_s"]=m.dt_s;out["delay_s"]=m.delay_s;out["iv_rank"]=m.iv_rank;
    out["structure"]=m.structure;out["parameter_count"]=m.parameter_count;
    out["iv_condition"]=m.iv_condition;out["prior_inertia"]=m.prior_inertia;
    out["training_samples"]=m.training_samples;out["validation_samples"]=m.validation_samples;
    out["rejected_samples"]=m.rejected_samples;out["duplicate_feedback"]=m.duplicate_feedback;
    out["command_samples"]=m.command_samples;out["resampled_samples"]=m.resampled_samples;
    out["feedback_interval_min_s"]=m.feedback_interval_min_s;out["feedback_interval_max_s"]=m.feedback_interval_max_s;
    out["observed_encoder_step_rad"]=m.observed_encoder_step_rad;
    out["fit_attempts"]=m.fit_attempts;out["rank_rejections"]=m.rank_rejections;
    out["plausibility_rejections"]=m.plausibility_rejections;out["prediction_rejections"]=m.prediction_rejections;
    out["independent_input_rms_nm"]=m.independent_input_rms_nm;out["model_input_rms_nm"]=m.input_rms_nm;
    out["position_noise_rad"]=m.position_noise_rad;out["measured_span_rad"]=m.measured_span_rad;
    out["training_span_rad"]=m.training_span_rad;out["validation_error_limit_rad"]=m.validation_error_limit_rad;
    if(m.training_samples) {
        out["training_rms_rad"]=m.training_rms_rad;
        out["arx"]=vector_node({m.arx_a1,m.arx_a2,m.arx_b0,m.arx_b1,m.bias});
    } else {out["training_rms_rad"]=YAML::Node();out["arx"]=YAML::Node();}
    const bool validation_evaluated=m.valid || m.prediction_rejections>0;
    out["validation_evaluated"]=validation_evaluated;
    if(validation_evaluated) {
        out["validation_rms_rad"]=m.validation_rms_rad;out["prior_validation_rms_rad"]=m.prior_validation_rms_rad;
    } else {out["validation_rms_rad"]=YAML::Node();out["prior_validation_rms_rad"]=YAML::Node();}
    out["predicted_cost_before"]=m.predicted_cost_before;
    out["predicted_cost_after"]=m.predicted_cost_after;out["step_fraction"]=m.applied_step_fraction;
    return out;
}
}
int main(int argc,char** argv) {
    try {
        fs::path task,output;std::string selected_record;long selected_event=-1;OnlineTrajectoryOptions options;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];const auto value=[&] {if(++i>=argc)throw std::runtime_error("missing value for "+arg);return std::string(argv[i]);};
            if(arg=="--task")task=value();else if(arg=="--output")output=value();else if(arg=="--record")selected_record=value();
            else if(arg=="--event-index")selected_event=std::stol(value());else if(arg=="--window-s")options.window_s=std::stod(value());
            else throw std::runtime_error("unknown argument "+arg);
        }
        if(task.empty() || output.empty())throw std::runtime_error("usage: --task DIR --output /tmp/report.json [--record CSV] [--event-index N] [--window-s 4]");
        task=fs::weakly_canonical(task);output=fs::absolute(output).lexically_normal();
        const auto canonical_output=fs::weakly_canonical(output);
        if(canonical_output.string().rfind("/tmp/",0)!=0)throw std::runtime_error("report output must be in /tmp");
        const auto source=YAML::LoadFile((task/"gain-candidate.json").string());
        const auto joints=source["joint_names"].as<std::vector<std::string>>();const auto n=joints.size();
        if(!n)throw std::runtime_error("empty task joint mapping");
        const auto initialization=source["model_initialization"];
        const auto inertia=numbers(initialization["inertia_max"],n,"model_initialization.inertia_max");
        const auto pmax=numbers(initialization["protocol_kp_max"],n,"protocol_kp_max");
        const auto dmax=numbers(initialization["protocol_kd_max"],n,"protocol_kd_max");
        const auto pstep=numbers(initialization["protocol_kp_quantum"],n,"protocol_kp_quantum");
        const auto dstep=numbers(initialization["protocol_kd_quantum"],n,"protocol_kd_quantum");
        std::vector<ModelAxisLimits> limits(n);std::vector<NoiseEstimate> noise(n);bool recorded_budget=false;
        for(std::size_t i=0;i<n;++i){limits[i].kp_max=pmax[i];limits[i].kd_max=dmax[i];limits[i].grid={pstep[i],dstep[i]};}
        for(const auto& experiment:source["experiments"]) {
            if(experiment["working_domain"] && experiment["working_domain"].size()==n && !recorded_budget) {
                for(std::size_t i=0;i<n;++i)limits[i].effort_budget=experiment["working_domain"][i]["feedback_effort_budget_nm"].as<double>();
                recorded_budget=true;
            }
            if(experiment["stage"] && experiment["stage"].as<std::string>()=="trajectory_noise" && experiment["axes"].size()==n)
                for(std::size_t i=0;i<n;++i) {
                    const auto a=experiment["axes"][i];noise[i].rms_rad=a["noise_rms_rad"].as<double>(0);
                    noise[i].encoder_step_rad=std::max(a["encoder_step_rad"].as<double>(0),a["effective_encoder_step_rad"].as<double>(0));
                }
        }
        if(!recorded_budget)throw std::runtime_error("task lacks recorded working-domain effort budgets; no default substituted");
        std::map<std::string,Rows> cached;std::map<std::string,std::size_t> reasons;std::size_t events=0,axes=0,valid=0,generated=0,admitted=0;
        YAML::Node report;report["task_id"]=source["task_id"];report["shadow_replay"]=true;
        report["interpretation"]="Archived feedback is reused only to replay each original decision independently; proposed gains were not physically executed";
        report["future_feedforward_assumption"]="last_executed_non_PD_torque_held_over_local_prediction_horizon";
        report["budget_source"]="recorded_working_domain_feedback_effort_budget";
        report["noise_source"]="recorded_static_noise_and_causal_position_code_spacing";
        long online_index=-1;
        for(const auto& event:source["candidate_comparisons"]) {
            if(!event["kind"] || event["kind"].as<std::string>()!="online_trajectory_PD_update")continue;
            ++online_index;if(selected_event>=0 && online_index!=selected_event)continue;
            const auto record=event["source_record_path"].as<std::string>();if(!selected_record.empty() && selected_record!=record)continue;
            if(!cached.count(record))cached.emplace(record,load_record(task/record,joints));
            const auto& all=cached.at(record);const double time=event["time_s"].as<double>();
            const Gains incumbent{numbers(event["previous_kp"],n,"event previous_kp"),numbers(event["previous_kd"],n,"event previous_kd")};
            Rows measured(n);auto causal_noise=noise;
            for(std::size_t i=0;i<n;++i) {
                std::vector<double> observed;
                for(const auto& row:all[i])if(row.time_s<=time) {
                    if(row.time_s>=time-8)measured[i].push_back(row);
                    if(row.new_measurement)observed.push_back(row.actual);
                }
                const double step=observed_encoder_step(observed);
                if(step>0)causal_noise[i].encoder_step_rad=causal_noise[i].encoder_step_rad>0?std::min(step,causal_noise[i].encoder_step_rad):step;
                if(measured[i].empty())throw std::runtime_error("event has no preceding mapped feedback");
            }
            std::vector<OnlineTrajectoryReference> future;
            for(double dt=-.06;dt<=.8+1e-9;dt+=.02) {
                OnlineTrajectoryReference f;f.time_s=time+dt;
                for(std::size_t i=0;i<n;++i) {
                    f.position.push_back(ref_at(all[i],f.time_s,false));f.velocity.push_back(ref_at(all[i],f.time_s,true));
                    f.feedforward_nm.push_back(measured[i].back().applied_feedforward_nm);
                }
                future.push_back(std::move(f));
            }
            auto purpose_options=options;auto purpose_limits=limits;
            const bool compliant=record.find("compliant")!=std::string::npos;
            if(compliant) {
                purpose_options.use=OnlineTrajectoryUse::Compliant;
                purpose_options.compliant_displacement_rad=source["gain_requirements"]["local_range_rad"].as<double>();
                // The online predictor receives the same purpose ceilings as
                // its archived caller, while final admission still uses the
                // original hardware protocol and feedback-effort boundaries.
                const auto rigid=source["targets"]["rigid_tracking"];
                if(rigid && rigid["kp"] && rigid["kd"]) {
                    const auto bounds=serial_arm::calibration_replay_supervisor::compliant_bounds(
                        numbers(rigid["kp"],n,"rigid kp"),numbers(rigid["kd"],n,"rigid kd"),pmax,dmax);
                    if(!bounds.valid)throw std::runtime_error("invalid archived compliant purpose ceiling");
                    for(std::size_t i=0;i<n;++i) {
                        purpose_limits[i].kp_max=quantize_gain(bounds.kp_ceiling[i],pstep[i],pmax[i]);
                        purpose_limits[i].kd_max=quantize_gain(bounds.kd_ceiling[i],dstep[i],dmax[i]);
                    }
                }
            }
            const auto decision=online_trajectory_candidate(incumbent,measured,future,inertia,purpose_limits,causal_noise,purpose_options);
            std::vector<double> E(n,std::numeric_limits<double>::quiet_NaN()),V=E,freshE(n),freshV(n);
            for(std::size_t i=0;i<n;++i) {
                double e=1e-9,v=0;std::size_t count=0;
                for(const auto& row:measured[i])if(row.new_measurement && row.actual_gains_known &&
                    std::abs(row.actual_kp-incumbent.kp[i])<1e-6 && std::abs(row.actual_kd-incumbent.kd[i])<1e-6) {
                    ++count;e=std::max(e,std::abs(row.reference-row.actual));v=std::max(v,std::abs(row.reference_velocity-row.velocity));
                }
                if(count>=30){E[i]=e;V[i]=v;}
                freshE[i]=measured[i].back().reference-measured[i].back().actual;freshV[i]=measured[i].back().reference_velocity-measured[i].back().velocity;
            }
            const auto admission=admit_complete_gain_candidate(incumbent,decision.gains,limits,E,V,freshE,freshV);
            YAML::Node item;item["event_index"]=online_index;item["time_s"]=time;item["record_path"]=record;
            item["use"]=compliant?"compliant":"tracking";
            if(compliant)item["compliant_displacement_rad"]=purpose_options.compliant_displacement_rad;
            item["historical_candidate_generated"]=event["candidate_generated"];item["historical_installed"]=event["installed"];
            item["candidate_generated"]=decision.changed;item["admission_evaluated"]=decision.changed;item["admission_allowed"]=decision.changed && admission.allowed;
            item["incumbent"]=gains_node(incumbent);item["proposal"]=gains_node(decision.gains);
            for(std::size_t i=0;i<n;++i){auto model=model_node(decision.models[i]);model["joint"]=joints[i];model["candidate_reason"]=decision.reasons[i];model["noise_encoder_step_rad"]=causal_noise[i].encoder_step_rad;item["models"].push_back(model);++axes;valid+=decision.models[i].valid;++reasons[decision.reasons[i]];}
            item["admission"]["axis_reasons"]=admission.axis_reasons;
            item["admission"]["incumbent_PD_envelope_nm"]=vector_node(admission.incumbent_PD_envelope_nm);
            item["admission"]["proposal_PD_envelope_nm"]=vector_node(admission.proposal_PD_envelope_nm);
            item["admission"]["feedback_effort_budget_nm"]=vector_node(admission.feedback_effort_budget_nm);
            report["events"].push_back(item);++events;generated+=decision.changed;admitted+=decision.changed && admission.allowed;
        }
        if(!events)throw std::runtime_error("no matching archived online events");
        report["summary"]["events"]=events;report["summary"]["axis_decisions"]=axes;report["summary"]["valid_models"]=valid;
        report["summary"]["generated_groups"]=generated;report["summary"]["admitted_groups"]=admitted;
        for(const auto& [reason,count]:reasons)report["summary"]["reasons"][reason]=count;
        // The wrapper converts this YAML intermediate to strict JSON.
        YAML::Emitter emitter;emitter<<report;
        std::ofstream file(output);if(!file)throw std::runtime_error("cannot open /tmp report");file<<emitter.c_str()<<'\n';
        std::cout<<"Replayed "<<events<<" original events; "<<valid<<"/"<<axes<<" supported models; "<<generated<<" proposed groups, "<<admitted<<" admitted groups\n";
    } catch(const std::exception& e){std::cerr<<"recorded online replay: "<<e.what()<<'\n';return 1;}
}
