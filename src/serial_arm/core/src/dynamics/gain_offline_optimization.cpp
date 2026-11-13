#include "serial_arm/dynamics/gain_offline_optimization.hpp"
#include <chrono>
#include <deque>

namespace serial_arm::servo_tuning {
namespace {
using Time=std::chrono::steady_clock;
double elapsed(Time::time_point start) {return std::chrono::duration<double>(Time::now()-start).count();}
Eigen::VectorXd vector(const std::vector<double>& values) {
    return Eigen::Map<const Eigen::VectorXd>(values.data(),values.size());
}
double encoded(double value,double step,double offset=0) {
    return step>0?std::floor((value-offset)/step)*step+offset:value;
}
Gains executable(Gains gains,const OfflineGainLimits& limits) {
    for(std::size_t axis=0;axis<gains.kp.size();++axis) {
        const auto& bound=limits.axes.at(axis);
        const double kp_ceiling=limits.kp_ceiling.empty()?bound.kp_max:limits.kp_ceiling.at(axis);
        const double kd_ceiling=limits.kd_ceiling.empty()?bound.kd_max:limits.kd_ceiling.at(axis);
        gains.kp[axis]=quantize_gain(std::clamp(gains.kp[axis],0.0,bound.kp_max),bound.grid.kp_step,bound.kp_max);
        gains.kd[axis]=quantize_gain(std::clamp(gains.kd[axis],0.0,bound.kd_max),bound.grid.kd_step,bound.kd_max);
        if(gains.kp[axis]>kp_ceiling)gains.kp[axis]=bound.grid.kp_step>0?
            std::floor(kp_ceiling/bound.grid.kp_step)*bound.grid.kp_step:kp_ceiling;
        if(gains.kd[axis]>kd_ceiling)gains.kd[axis]=bound.grid.kd_step>0?
            std::floor(kd_ceiling/bound.grid.kd_step)*bound.grid.kd_step:kd_ceiling;
    }
    return gains;
}
bool shape_valid(const OfflineGainPath& path,const OfflineGainModel& model,const Gains& gains,const OfflineGainLimits& limits) {
    const auto count=static_cast<Eigen::Index>(gains.kp.size());
    if(!count || gains.kd.size()!=gains.kp.size() || limits.axes.size()!=gains.kp.size() ||
       (!limits.kp_ceiling.empty() && limits.kp_ceiling.size()!=gains.kp.size()) ||
       (!limits.kd_ceiling.empty() && limits.kd_ceiling.size()!=gains.kp.size()) ||
       path.steps.size()<2 || !std::isfinite(path.period_s) || path.period_s<=0)return false;
    for(const auto* value:{&path.initial_position,&path.initial_velocity,&path.lower,&path.upper,
        &model.coulomb,&model.viscous,&model.bias,&model.rotor,&model.gravity_error,&limits.lower,&limits.upper,
        &limits.velocity,&limits.effort,&limits.position_step,&limits.velocity_step,&limits.torque_step})
        if(value->size()!=count || !value->allFinite())return false;
    if((model.coulomb.array()<0).any() || (model.viscous.array()<0).any() || (model.rotor.array()<0).any() ||
       (limits.velocity.array()<=0).any() || (limits.effort.array()<=0).any() ||
       (limits.position_step.array()<0).any() || (limits.velocity_step.array()<0).any() || (limits.torque_step.array()<0).any() ||
       (path.lower.array()>path.upper.array()).any())return false;
    for(const auto& step:path.steps) {
        for(const auto* value:{&step.reference,&step.reference_velocity,&step.actual,&step.velocity,&step.feedforward,&step.gravity})
            if(value->size()!=count || !value->allFinite())return false;
        for(const auto* matrix:{&step.mass,&step.coriolis,&step.gravity_gradient})
            if(matrix->rows()!=count || matrix->cols()!=count || !matrix->allFinite())return false;
    }
    for(std::size_t axis=0;axis<gains.kp.size();++axis)
        if(!std::isfinite(gains.kp[axis]) || gains.kp[axis]<0 || !std::isfinite(gains.kd[axis]) || gains.kd[axis]<0)return false;
    return true;
}
Eigen::VectorXd prediction_residual(const OfflineGainPath& path,const OfflineGainTrace& trace) {
    if(!trace.complete)return {};
    const auto count=path.initial_position.size();
    const std::size_t stride=8,start=std::min<std::size_t>(60,path.steps.size()/8);
    Eigen::VectorXd residual(((path.steps.size()-1-start)/stride+1)*count);
    Eigen::Index offset=0;
    for(std::size_t index=start;index<path.steps.size();index+=stride) {
        residual.segment(offset,count)=trace.position.row(index).transpose()-path.steps[index].actual;
        offset+=count;
    }
    return residual;
}
}

OfflineGainTrustRegion offline_gain_local_trust_region(OfflineGainUse use,const Gains& seed,
    const OfflineGainLimits& limits) {
    if(seed.kp.empty() || seed.kd.size()!=seed.kp.size() || limits.axes.size()!=seed.kp.size() ||
       (!limits.kp_ceiling.empty() && limits.kp_ceiling.size()!=seed.kp.size()) ||
       (!limits.kd_ceiling.empty() && limits.kd_ceiling.size()!=seed.kp.size()))
        throw std::invalid_argument("offline gain trust region mapping mismatch");
    for(std::size_t i=0;i<seed.kp.size();++i) {
        const auto& x=limits.axes[i];
        for(double value:{seed.kp[i],seed.kd[i],x.grid.kp_step,x.grid.kd_step})
            if(!std::isfinite(value) || value<0)throw std::invalid_argument("invalid offline gain trust region seed or grid");
        for(double value:{x.kp_max,x.kd_max})
            if(!std::isfinite(value) || value<=0)throw std::invalid_argument("invalid offline gain protocol capability");
    }
    const auto anchor=executable(seed,limits);
    OfflineGainTrustRegion region;region.lower=anchor;region.upper=anchor;
    for(std::size_t i=0;i<seed.kp.size();++i)for(unsigned channel=0;channel<2;++channel) {
        const bool p=channel==0;const auto& x=limits.axes[i];
        const double base=p?anchor.kp[i]:anchor.kd[i],step=p?x.grid.kp_step:x.grid.kd_step;
        const double protocol=p?x.kp_max:x.kd_max;
        const double ceiling=std::min(protocol,p?(limits.kp_ceiling.empty()?protocol:limits.kp_ceiling[i]):
            (limits.kd_ceiling.empty()?protocol:limits.kd_ceiling[i]));
        if(!std::isfinite(ceiling) || ceiling<0)throw std::invalid_argument("invalid offline gain execution ceiling");
        double lower=quantize_gain(.75*base,step,protocol,true);
        double upper=quantize_gain(std::min(ceiling,(p?1.5:1.25)*base),step,protocol);
        // Only a nearest adjacent executable code may supplement an interval
        // which would otherwise contain no change in that direction.
        if(step>0 && base>0 && lower>=base)lower=quantize_gain(std::max(0.0,base-step),step,protocol);
        if(step>0 && upper<=base && base<ceiling)
            upper=quantize_gain(std::min(ceiling,base+step),step,protocol);
        if(p && use==OfflineGainUse::Drag)lower=upper=0;
        (p?region.lower.kp[i]:region.lower.kd[i])=lower;
        (p?region.upper.kp[i]:region.upper.kd[i])=upper;
    }
    return region;
}

OfflineGainTrace simulate_offline_gain_path(const OfflineGainPath& path,const OfflineGainModel& model,
    const Gains& gains,const OfflineGainLimits& limits,const Eigen::MatrixXd& independent_input,
    double actuator_lag_s,double velocity_lag_s,double inertia_scale,unsigned subdivisions,
    const OfflineGainCheck& check,const OfflineGainDynamics& dynamics,const OfflineGainSimulationOptions& options) {
    OfflineGainTrace result;
    if(!shape_valid(path,model,gains,limits) || !subdivisions || !std::isfinite(inertia_scale) || inertia_scale<=0 ||
       !std::isfinite(actuator_lag_s) || actuator_lag_s<=0 || !std::isfinite(velocity_lag_s) || velocity_lag_s<=0)return result;
    for(double timing:{options.inner_pd_period_s,options.position_feedback_delay_s,options.velocity_feedback_delay_s})
        if(!std::isfinite(timing) || timing<0)return result;
    const auto count=path.initial_position.size(),samples=static_cast<Eigen::Index>(path.steps.size());
    if(independent_input.size() && (independent_input.rows()!=samples || independent_input.cols()!=count || !independent_input.allFinite()))return result;
    result.position=Eigen::MatrixXd::Zero(samples,count);result.velocity=result.position;
    result.torque=result.position;result.feedback=result.position;
    Eigen::VectorXd position=path.initial_position,velocity=path.initial_velocity,filtered=velocity;
    Eigen::VectorXd motor=Eigen::VectorXd::Zero(count);
    result.position.row(0)=position;result.velocity.row(0)=velocity;
    const double step=path.period_s/subdivisions;
    const double motor_fraction=1-std::exp(-step/actuator_lag_s),velocity_fraction=1-std::exp(-step/velocity_lag_s);
    const Eigen::VectorXd stiffness=vector(gains.kp),damping=vector(gains.kd);
    const auto total_ticks=static_cast<std::size_t>(samples-1)*subdivisions;
    const auto ticks=[&](double seconds) {
        return static_cast<std::size_t>(std::min(static_cast<double>(total_ticks+1),std::ceil(seconds/step-1e-12)));
    };
    const auto pd_ticks=std::max<std::size_t>(1,ticks(options.inner_pd_period_s));
    const auto position_delay=ticks(options.position_feedback_delay_s),velocity_delay=ticks(options.velocity_feedback_delay_s);
    const auto history_size=std::max(position_delay,velocity_delay)+1;
    std::deque<Eigen::VectorXd> positions,velocities;
    Eigen::VectorXd feedback=Eigen::VectorXd::Zero(count);
    for(Eigen::Index index=1;index<samples;++index) {
        if(index%128==1 && check && !check())return result;
        auto local=path.steps[index-1];
        Eigen::VectorXd observed=index>1?result.position.row(index-2).transpose().eval():path.initial_position;
        Eigen::VectorXd feedforward=local.feedforward+
            (local.feedforward_gradient.size()?local.feedforward_gradient:local.gravity_gradient)*(observed-local.actual);
        if(dynamics && !dynamics(position,velocity,observed,local))return result;
        if(dynamics)feedforward=local.feedforward;
        Eigen::MatrixXd inertia=local.mass;
        if(options.include_fitted_rotor_inertia)inertia.diagonal()+=model.rotor;
        inertia*=inertia_scale;
        Eigen::MatrixXd implicit=inertia;implicit.diagonal()+=step*model.viscous;
        const Eigen::LDLT<Eigen::MatrixXd> factor(implicit);
        if(factor.info()!=Eigen::Success || !factor.isPositive())return result;
        const Eigen::MatrixXd inverse=factor.solve(Eigen::MatrixXd::Identity(count,count));
        if(!inverse.allFinite())return result;
        Eigen::VectorXd reference=local.reference,reference_velocity=local.reference_velocity;
        Eigen::VectorXd input=Eigen::VectorXd::Zero(count);
        if(independent_input.size())input=independent_input.row(index-1);
        for(Eigen::Index axis=0;axis<count;++axis) {
            reference_velocity(axis)=encoded(reference_velocity(axis),limits.velocity_step(axis));
            feedforward(axis)=encoded(feedforward(axis),limits.torque_step(axis),-limits.effort(axis));
        }
        const Eigen::VectorXd gravity_at_start=dynamics?local.gravity:
            (local.gravity+local.gravity_gradient*(position-local.reference)).eval();
        const Eigen::VectorXd position_start=position;
        for(unsigned substep=0;substep<subdivisions;++substep) {
            positions.push_back(position);velocities.push_back(velocity);
            if(positions.size()>history_size){positions.pop_front();velocities.pop_front();}
            Eigen::VectorXd sensed_position=positions[position_delay<positions.size()?positions.size()-1-position_delay:0];
            Eigen::VectorXd sensed_velocity=velocities[velocity_delay<velocities.size()?velocities.size()-1-velocity_delay:0];
            for(Eigen::Index axis=0;axis<count;++axis) {
                if(options.quantize_position_feedback && limits.position_step(axis)>0)
                    sensed_position(axis)=std::round(sensed_position(axis)/limits.position_step(axis))*limits.position_step(axis);
                if(options.quantize_velocity_feedback && limits.velocity_step(axis)>0)
                    sensed_velocity(axis)=std::round(sensed_velocity(axis)/limits.velocity_step(axis))*limits.velocity_step(axis);
            }
            filtered+=velocity_fraction*(sensed_velocity-filtered);
            const auto tick=static_cast<std::size_t>(index-1)*subdivisions+substep;
            if(tick%pd_ticks==0)feedback=stiffness.cwiseProduct(reference-sensed_position)+
                damping.cwiseProduct(reference_velocity-filtered);
            if(tick==0)motor=feedback+feedforward;
            Eigen::VectorXd requested=feedback+feedforward+input;
            for(Eigen::Index axis=0;axis<count;++axis) {
                if(std::abs(requested(axis))>limits.effort(axis))result.safe=false;
                requested(axis)=std::clamp(requested(axis),-limits.effort(axis),limits.effort(axis));
                if(std::abs(feedback(axis))>limits.axes[axis].effort_budget+1e-9)result.safe=false;
            }
            motor+=motor_fraction*(requested-motor);
            const Eigen::VectorXd gravity=gravity_at_start+local.gravity_gradient*(position-position_start);
            const Eigen::VectorXd coriolis=dynamics && local.coriolis_force.size()==count?
                local.coriolis_force:(local.coriolis*velocity).eval();
            const Eigen::VectorXd resistance=inertia_scale*coriolis+
                gravity.cwiseProduct(Eigen::VectorXd::Ones(count)+model.gravity_error)+model.bias;
            velocity=inverse*(inertia*velocity+step*(motor-resistance));
            Eigen::VectorXd impulse=Eigen::VectorXd::Zero(count);
            for(unsigned sweep=0;sweep<12;++sweep) {
                double movement=0;
                for(Eigen::Index axis=0;axis<count;++axis) {
                    const double bound=step*model.coulomb(axis);
                    const double next=std::clamp(impulse(axis)+velocity(axis)/inverse(axis,axis),-bound,bound);
                    const double delta=next-impulse(axis);impulse(axis)=next;
                    velocity-=inverse.col(axis)*delta;movement+=std::abs(delta);
                }
                if(movement<1e-11)break;
            }
            position+=step*velocity;
            if(!position.allFinite() || !velocity.allFinite() || position.cwiseAbs().maxCoeff()>30 || velocity.cwiseAbs().maxCoeff()>300)return result;
            if((position.array()<path.lower.array()).any() || (position.array()>path.upper.array()).any() ||
               (position.array()<limits.lower.array()).any() || (position.array()>limits.upper.array()).any() ||
               (velocity.cwiseAbs().array()>limits.velocity.array()).any())result.safe=false;
        }
        result.position.row(index)=position;result.velocity.row(index)=velocity;
        result.torque.row(index)=motor;result.feedback.row(index)=feedback;
    }
    result.complete=true;return result;
}

OfflineGainModel fit_offline_gain_model(const OfflineGainPath& measured,const Gains& actual_gains,
    const OfflineGainLimits& limits,double budget_s,const OfflineGainCheck& check) {
    const auto start=Time::now();const auto count=measured.initial_position.size();
    OfflineGainModel model;model.coulomb=Eigen::VectorXd::Zero(count);model.viscous=model.coulomb;
    model.bias=model.coulomb;model.rotor=model.coulomb;model.gravity_error=model.coulomb;
    if(measured.steps.size()<150 || !shape_valid(measured,model,actual_gains,limits) || !std::isfinite(budget_s) || budget_s<=0) {
        model.reason="insufficient_or_invalid_complete_dynamic_model_record";return model;
    }
    model.axis_fitted_samples.assign(count,0);bool complete_dynamic_evidence=true;
    Eigen::VectorXd gravity_excitation=Eigen::VectorXd::Zero(count);
    for(const auto& row:measured.steps)gravity_excitation=gravity_excitation.cwiseMax(row.gravity.cwiseAbs());
    for(Eigen::Index axis=0;axis<count;++axis) {
        Eigen::Matrix<double,5,5> normal=Eigen::Matrix<double,5,5>::Identity()*1e-5;
        Eigen::Matrix<double,5,1> target=Eigen::Matrix<double,5,1>::Zero();std::size_t samples=0;
        double min_velocity=std::numeric_limits<double>::infinity(),max_velocity=-min_velocity;
        double min_acceleration=min_velocity,max_acceleration=-min_velocity;
        for(std::size_t index=2;index+2<measured.steps.size();++index) {
            const auto& row=measured.steps[index];
            const Eigen::VectorXd acceleration=(measured.steps[index+2].velocity-measured.steps[index-2].velocity)/(4*measured.period_s);
            const double velocity=row.velocity(axis);
            if(row.torque.size()!=count || !row.torque.allFinite() || std::abs(velocity)<.04 || std::abs(acceleration(axis))>2)continue;
            Eigen::Matrix<double,5,1> design;
            const double gravity=row.measured_gravity.size()==count?row.measured_gravity(axis):row.gravity(axis);
            design<<std::copysign(1.0,velocity),velocity,1,acceleration(axis),gravity;
            const double nominal=row.measured_inverse_dynamics.size()==count?row.measured_inverse_dynamics(axis):
                (row.mass*acceleration+row.coriolis*row.velocity+row.gravity)(axis);
            const double residual=row.torque(axis)-nominal;
            const double weight=1/std::max(1.0,std::abs(residual)/std::max(.02,.1*limits.effort(axis)));
            normal+=weight*design*design.transpose();target+=weight*design*residual;++samples;
            min_velocity=std::min(min_velocity,velocity);max_velocity=std::max(max_velocity,velocity);
            min_acceleration=std::min(min_acceleration,acceleration(axis));max_acceleration=std::max(max_acceleration,acceleration(axis));
        }
        if(samples>=30) {
            const Eigen::Matrix<double,5,1> parameters=normal.ldlt().solve(target);
            if(parameters.allFinite()) {
                model.coulomb(axis)=std::clamp(parameters(0),0.0,.25*limits.effort(axis));
                model.viscous(axis)=std::clamp(parameters(1),0.0,5.0);
                model.bias(axis)=std::clamp(parameters(2),-.1*limits.effort(axis),.1*limits.effort(axis));
                model.rotor(axis)=std::clamp(parameters(3),0.0,.3);
                model.gravity_error(axis)=gravity_excitation(axis)>1e-8?std::clamp(parameters(4),-.2,.2):0;
            }
        }
        model.fitted_samples+=samples;
        model.axis_fitted_samples[axis]=samples;
        if(samples<30 || max_velocity-min_velocity<=std::max(4*limits.velocity_step(axis),1e-4) ||
           max_acceleration-min_acceleration<=1e-4)complete_dynamic_evidence=false;
    }
    if(!complete_dynamic_evidence) {
        model.reason="insufficient_dynamic_samples_or_velocity_acceleration_variation_for_complete_model";return model;
    }
    const auto allowed=[&] {return elapsed(start)<budget_s && (!check || check());};
    auto trace=simulate_offline_gain_path(measured,model,actual_gains,limits,{},.001,.0005,1,10,allowed);
    auto residual=prediction_residual(measured,trace);
    if(!residual.size()){model.reason="same_record_prediction_incomplete";return model;}
    const auto initial=model;
    for(unsigned iteration=0;iteration<3 && allowed();++iteration) {
        Eigen::MatrixXd jacobian(residual.size(),5*count);jacobian.setZero();
        for(Eigen::Index parameter=0;parameter<5*count && allowed();++parameter) {
            auto probe=model;
            std::array<Eigen::VectorXd*,5> fields{&probe.coulomb,&probe.viscous,&probe.bias,&probe.rotor,&probe.gravity_error};
            auto& value=(*fields[parameter/count])(parameter%count);
            const double difference=std::max(1e-4,.003*std::max(.05,std::abs(value)));value+=difference;
            const auto prediction=simulate_offline_gain_path(measured,probe,actual_gains,limits,{},.001,.0005,1,10,allowed);
            const auto shifted=prediction_residual(measured,prediction);++model.refinement_evaluations;
            if(shifted.size()!=residual.size())break;
            jacobian.col(parameter)=(shifted-residual)/difference;
        }
        if(!allowed())break;
        Eigen::MatrixXd normal=jacobian.transpose()*jacobian;normal.diagonal().array()+=.01;
        const Eigen::VectorXd increment=-normal.ldlt().solve(jacobian.transpose()*residual);
        if(!increment.allFinite())break;
        bool improved=false;
        for(double fraction:{1.0,.5,.25}) {
            auto proposal=model;
            std::array<Eigen::VectorXd*,5> fields{&proposal.coulomb,&proposal.viscous,&proposal.bias,&proposal.rotor,&proposal.gravity_error};
            for(Eigen::Index axis=0;axis<count;++axis) {
                for(Eigen::Index group=0;group<5;++group)(*fields[group])(axis)+=fraction*increment(group*count+axis);
                proposal.coulomb(axis)=std::clamp(proposal.coulomb(axis),0.0,.25*limits.effort(axis));
                proposal.viscous(axis)=std::clamp(proposal.viscous(axis),0.0,5.0);
                proposal.bias(axis)=std::clamp(proposal.bias(axis),initial.bias(axis)-.25,initial.bias(axis)+.25);
                proposal.rotor(axis)=std::clamp(proposal.rotor(axis),0.0,.3);
                proposal.gravity_error(axis)=std::clamp(proposal.gravity_error(axis),-.2,.2);
                if(gravity_excitation(axis)<=1e-8)proposal.gravity_error(axis)=0;
            }
            auto predicted=simulate_offline_gain_path(measured,proposal,actual_gains,limits,{},.001,.0005,1,10,allowed);
            auto proposed_residual=prediction_residual(measured,predicted);++model.refinement_evaluations;
            if(proposed_residual.size()==residual.size() && proposed_residual.squaredNorm()<residual.squaredNorm()) {
                model=std::move(proposal);residual=std::move(proposed_residual);trace=std::move(predicted);improved=true;break;
            }
        }
        if(!improved)break;
    }
    model.prediction_rms_rad=Eigen::VectorXd::Zero(count);
    for(std::size_t index=0;index<measured.steps.size();++index)
        model.prediction_rms_rad+=(trace.position.row(index).transpose()-measured.steps[index].actual).array().square().matrix();
    model.prediction_rms_rad=(model.prediction_rms_rad/measured.steps.size()).array().sqrt();
    model.available=trace.complete && model.prediction_rms_rad.allFinite();
    model.reason=model.available?"same_record_low_frequency_prediction_fit_only_no_identified_stability":
        "same_record_prediction_incomplete_or_nonfinite";
    return model;
}

OfflineGainEvaluation evaluate_offline_gain_use(OfflineGainUse use,const OfflineGainPath& tracking,
    const OfflineGainPath& holding,const OfflineGainModel& model,const Gains& gains,
    const OfflineGainLimits& limits,const Requirements& requirements,const Eigen::VectorXd& amplitudes,
    const OfflineGainCheck& check,bool perturbations,const OfflineGainDynamics& dynamics,const OfflineGainSimulationOptions& simulation_options) {
    OfflineGainEvaluation evaluation;const auto count=static_cast<Eigen::Index>(gains.kp.size());
    if(!model.available || !count || amplitudes.size()!=count || !amplitudes.allFinite() ||
       (amplitudes.array()<0).any())return evaluation;
    if(perturbations) {
        // These are declared stress assumptions, not measured MIT firmware
        // timing. Host feedback age does not set an inner-controller delay.
        auto sampled=simulation_options;
        sampled.inner_pd_period_s=std::max(.001,sampled.inner_pd_period_s);
        sampled.position_feedback_delay_s=std::max(.001,sampled.position_feedback_delay_s);
        sampled.velocity_feedback_delay_s=std::max(.001,sampled.velocity_feedback_delay_s);
        sampled.quantize_position_feedback=sampled.quantize_velocity_feedback=true;
        auto first=evaluate_offline_gain_use(use,tracking,holding,model,gains,limits,requirements,
            amplitudes,check,false,dynamics,sampled);
        auto urdf=sampled;urdf.include_fitted_rotor_inertia=false;
        auto second=evaluate_offline_gain_use(use,tracking,holding,model,gains,limits,requirements,
            amplitudes,check,false,dynamics,urdf);
        first.cost=std::max(first.cost,second.cost);first.violation+=second.violation;
        first.complete=first.complete && second.complete;first.feasible=first.feasible && second.feasible;
        first.simulation_scenarios+=second.simulation_scenarios;
        first.includes_urdf_only_inertia=true;
        for(std::size_t i=0;i<std::min(first.peak_error_rad.size(),second.peak_error_rad.size());++i)
            first.peak_error_rad[i]=std::max(first.peak_error_rad[i],second.peak_error_rad[i]);
        return first;
    }
    evaluation.cost=0;evaluation.violation=0;evaluation.complete=true;
    evaluation.simulation_scenarios=1;evaluation.includes_urdf_only_inertia=!simulation_options.include_fitted_rotor_inertia;
    const bool sampled=simulation_options.inner_pd_period_s>0 || simulation_options.position_feedback_delay_s>0 ||
        simulation_options.velocity_feedback_delay_s>0;
    const double lag=sampled?.003:.001,velocity_lag=sampled?.001:.0005;
    const auto include=[&](const OfflineGainPath& path,const Eigen::MatrixXd& input) {
        auto trace=simulate_offline_gain_path(path,model,gains,limits,input,lag,velocity_lag,1,10,check,dynamics,simulation_options);
        evaluation.complete=evaluation.complete && trace.complete;
        if(!trace.complete)evaluation.violation+=100;
        else if(!trace.safe)evaluation.violation+=1;
        return trace;
    };
    if(use==OfflineGainUse::Tracking || use==OfflineGainUse::Compliant) {
        const auto trace=include(tracking,{});
        if(trace.complete) {
            for(Eigen::Index axis=0;axis<count;++axis) {
                double peak=0,square=0;
                for(std::size_t index=0;index<tracking.steps.size();++index) {
                    const double error=trace.position(index,axis)-tracking.steps[index].reference(axis);
                    peak=std::max(peak,std::abs(error));square+=error*error;
                }
                evaluation.peak_error_rad.push_back(peak);
                evaluation.cost+=(peak*peak+.5*square/tracking.steps.size())*std::pow(180/pi,2)/count;
            }
        }
    }
    if(use==OfflineGainUse::RigidHold || use==OfflineGainUse::Compliant) {
        Eigen::MatrixXd input=Eigen::MatrixXd::Zero(holding.steps.size(),count);
        for(std::size_t index=0;index<holding.steps.size();++index)
            for(Eigen::Index axis=0;axis<count;++axis) {
                const double phase=(holding.steps[index].time_s-(.8+2*axis))/.5;
                if(phase>=0 && phase<1)input(index,axis)=amplitudes(axis)*std::pow(std::sin(pi*phase),2);
            }
        const auto trace=include(holding,input);
        if(trace.complete)for(Eigen::Index axis=0;axis<count;++axis) {
            double baseline=0;std::size_t baseline_samples=0;
            for(std::size_t index=0;index<holding.steps.size();++index) {
                const double time=holding.steps[index].time_s-(.8+2*axis);
                if(time>=-.2 && time<=-.05){baseline+=trace.position(index,axis);++baseline_samples;}
            }
            if(!baseline_samples){evaluation.violation+=100;continue;}
            baseline/=baseline_samples;
            double yielding=0,recovery=0,speed=0,last_unsettled_s=0;
            for(std::size_t index=0;index<holding.steps.size();++index) {
                const double time=holding.steps[index].time_s-(.8+2*axis);
                const double error=std::abs(trace.position(index,axis)-baseline);
                if(time>=0 && time<=.5)yielding=std::max(yielding,error);
                if(time>=1.5 && time<=2) {
                    recovery=std::max(recovery,error);speed=std::max(speed,std::abs(trace.velocity(index,axis)));
                }
                if(time>=.5 && time<=2 && (error>.008 || std::abs(trace.velocity(index,axis))>.01))
                    last_unsettled_s=time-.5;
            }
            const double floor=std::max(3*limits.position_step(axis),.0002);
            if(use==OfflineGainUse::Compliant) {
                evaluation.violation+=std::max(0.0,(floor-yielding)/floor);
                evaluation.violation+=std::max(0.0,(yielding-requirements.local_range_rad)/requirements.local_range_rad);
            }
            evaluation.violation+=std::max(0.0,(recovery-.008)/.008)+std::max(0.0,(speed-.01)/.01);
            const double settle=use==OfflineGainUse::RigidHold?requirements.settling_time_s:1.5;
            evaluation.violation+=std::max(0.0,(last_unsettled_s-settle)/settle);
            const auto quiet=std::min<Eigen::Index>(trace.position.rows(),static_cast<Eigen::Index>(.5/holding.period_s));
            const double quiet_error=(trace.position.bottomRows(quiet).col(axis).array()-holding.initial_position(axis)).abs().maxCoeff();
            const double quiet_speed=trace.velocity.bottomRows(quiet).col(axis).cwiseAbs().maxCoeff();
            evaluation.violation+=std::max(0.0,(quiet_error-requirements.hold_error_rad)/requirements.hold_error_rad)+
                std::max(0.0,(quiet_speed-requirements.hold_velocity_rad_s)/requirements.hold_velocity_rad_s);
            evaluation.cost+=(recovery*recovery+(use==OfflineGainUse::RigidHold?yielding*yielding:0))*std::pow(180/pi,2)/count;
        }
    }
    if(use==OfflineGainUse::Drag) {
        for(double stiffness:gains.kp)if(stiffness!=0)evaluation.violation+=100;
        for(Eigen::Index axis=-1;axis<count;++axis) {
            auto release=holding;release.steps.resize(std::min<std::size_t>(release.steps.size(),601));
            if(axis>=0)release.initial_velocity(axis)=.02;
            const auto trace=include(release,{});
            if(!trace.complete)continue;
            const double speed=trace.velocity.bottomRows(std::min<Eigen::Index>(40,trace.velocity.rows())).cwiseAbs().maxCoeff();
            const double drift=(trace.position.bottomRows(std::min<Eigen::Index>(100,trace.position.rows())).colwise().maxCoeff()-
                trace.position.bottomRows(std::min<Eigen::Index>(100,trace.position.rows())).colwise().minCoeff()).maxCoeff();
            evaluation.violation+=std::max(0.0,(speed-.003)/.003)+std::max(0.0,(drift-.004)/.004);
            if(axis<0)evaluation.violation+=std::max(0.0,((trace.position.row(trace.position.rows()-1).transpose()-release.initial_position).cwiseAbs().maxCoeff()-.004)/.004);
            evaluation.cost+=speed*speed+drift*drift;
        }
        evaluation.cost+=.0001*vector(gains.kd).squaredNorm();
    }
    evaluation.feasible=evaluation.complete && evaluation.violation<=1e-9;
    if(!evaluation.complete)evaluation.cost=std::numeric_limits<double>::infinity();
    return evaluation;
}

OfflineGainOptimization optimize_offline_gain_use(OfflineGainUse use,const OfflineGainPath& tracking,
    const OfflineGainPath& holding,const OfflineGainModel& model,const Gains& seed,
    const std::vector<std::vector<double>>& mass_basis,const OfflineGainLimits& limits,
    const Requirements& requirements,const Eigen::VectorXd& amplitudes,double budget_s,const OfflineGainCheck& check) {
    (void)mass_basis; // Kept in the public signature for existing callers.
    OfflineGainOptimization result;result.trust_region=offline_gain_local_trust_region(use,seed,limits);
    result.gains=executable(seed,limits);const auto start=Time::now();
    if(use==OfflineGainUse::Drag)std::fill(result.gains.kp.begin(),result.gains.kp.end(),0);
    const auto original=result.gains;
    if(budget_s<=0 || !model.available)return result;
    const auto allowed=[&] {return elapsed(start)<budget_s && (!check || check());};
    auto current=evaluate_offline_gain_use(use,tracking,holding,model,result.gains,limits,requirements,amplitudes,allowed);
    result.baseline=current;result.selected=current;++result.evaluations;
    const auto evaluate=[&](const Gains& proposal) {
        if(!allowed())return false;
        auto actual=executable(proposal,limits);
        for(std::size_t i=0;i<actual.kp.size();++i) {
            actual.kp[i]=std::clamp(actual.kp[i],result.trust_region.lower.kp[i],result.trust_region.upper.kp[i]);
            actual.kd[i]=std::clamp(actual.kd[i],result.trust_region.lower.kd[i],result.trust_region.upper.kd[i]);
        }
        auto value=evaluate_offline_gain_use(use,tracking,holding,model,actual,limits,requirements,amplitudes,allowed);
        ++result.evaluations;
        const bool better=value.feasible!=result.selected.feasible?value.feasible:
            value.violation!=result.selected.violation?value.violation<result.selected.violation:value.cost<result.selected.cost;
        if(value.complete && better) {
            result.gains=actual;result.selected=std::move(value);return true;
        }
        return false;
    };
    double mesh=std::log(1.25);
    for(unsigned iteration=0;iteration<40 && mesh>.015 && allowed();++iteration) {
        const auto center=result.gains;bool improved=false;
        for(std::size_t dimension=use==OfflineGainUse::Drag?center.kp.size():0;dimension<2*center.kp.size() && allowed();++dimension)
            for(double direction:{-1.0,1.0}) {
                auto proposal=center;const auto axis=dimension%center.kp.size();
                auto& value=dimension<center.kp.size()?proposal.kp[axis]:proposal.kd[axis];
                const auto step=dimension<center.kp.size()?limits.axes[axis].grid.kp_step:limits.axes[axis].grid.kd_step;
                value=std::max(0.0,value*std::exp(direction*mesh)+direction*step);
                improved=evaluate(proposal) || improved;
            }
        mesh=improved?std::min(std::log(1.25),mesh*1.2):mesh*.5;
    }
    result.elapsed_s=elapsed(start);result.budget_exhausted=result.elapsed_s>=budget_s;
    result.available=result.selected.feasible;result.changed=result.available &&
        (result.gains.kp!=original.kp || result.gains.kd!=original.kd);return result;
}
}
