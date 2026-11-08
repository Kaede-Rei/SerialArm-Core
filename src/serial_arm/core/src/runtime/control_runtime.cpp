#include "serial_arm/runtime/control_runtime.hpp"
#include "serial_arm/core/park_policy.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <thread>
#include <type_traits>

namespace serial_arm {
namespace {
RobotFault rejected(RobotErr code, JointCtrllerErr controller = JointCtrllerErr::CMD_NOT_ALLOWED_IN_MODE) {
    RobotFault fault; fault.code=code; fault.ctrller_err=controller; return fault;
}
bool finite(const JointVector& values, std::size_t size) {
    return values.size()==size && std::all_of(values.begin(), values.end(), [](double v){return std::isfinite(v);});
}
}

tl::expected<void, RobotFault> ControlRuntime::submit_reference(
    Robot& robot, JointCmd command, Robot::TimePoint received_at, MotionSource source) {
    if(robot.get_state()!=RobotState::ACTIVE)
        return tl::make_unexpected(rejected(robot.get_state()==RobotState::FAULT ? RobotErr::FAULTED : RobotErr::NOT_ACTIVE));
    const auto mode=robot.get_impedance_mode();
    if(mode!=JointImpedanceMode::RIGID_TRACKING && mode!=JointImpedanceMode::COMPLIANT_TRACKING)
        return tl::make_unexpected(rejected(RobotErr::CTRLLER_FAILED));
    const auto n=robot.get_joint_state().pos.size();
    const bool valid=std::visit([&](const auto& cmd){
        using T=std::decay_t<decltype(cmd)>;
        if(!finite(cmd.pos,n))return false;
        if constexpr(!std::is_same_v<T,JointPosCmd>) { if(!finite(cmd.vel,n))return false; }
        if constexpr(std::is_same_v<T,JointPosVelTorCmd>) { if(!finite(cmd.tor,n))return false; }
        return true;
    },command);
    if(!valid || source==MotionSource::NONE)
        return tl::make_unexpected(rejected(RobotErr::CTRLLER_FAILED,JointCtrllerErr::INVALID_CMD_VALUE));
    std::lock_guard<std::mutex> lock(command_mutex_);
    if(source_!=MotionSource::NONE && source_!=source)
        return tl::make_unexpected(rejected(RobotErr::MOTION_SOURCE_BUSY));
    source_=source;
    pending_=PendingReference{std::move(command),received_at};
    return {};
}

tl::expected<void, RobotFault> ControlRuntime::set_impedance_mode(Robot& robot, JointImpedanceMode mode, Robot::TimePoint now, HoldTargetPolicy target_policy) {
    const auto result=robot.set_impedance_mode(mode,now,target_policy);
    if(result) {
        // Legacy drag requests explicitly select native MIT interaction; this
        // adapter policy is separate from Core's configurable reference law
        if(mode==JointImpedanceMode::COMPLIANT_DRAG) {
            if(!drag_suspension_)drag_suspension_=robot.is_admittance_suspended();
            robot.set_admittance_suspended(true);
        } else if(drag_suspension_) {
            robot.set_admittance_suspended(*drag_suspension_);
            drag_suspension_.reset();
        }
        clear_commands();
    }
    return result;
}

void ControlRuntime::clear_commands() {
    std::lock_guard<std::mutex> lock(command_mutex_);
    pending_.reset();source_=MotionSource::NONE;
}
MotionSource ControlRuntime::source() const {
    std::lock_guard<std::mutex> lock(command_mutex_);return source_;
}
RuntimeSnapshot ControlRuntime::snapshot() const {
    std::lock_guard<std::mutex> lock(snapshot_mutex_);return snapshot_;
}

RuntimeSnapshot ControlRuntime::step(Robot& robot, Robot::TimePoint now) {
    if(executing_.exchange(true))throw std::logic_error("control runtime already has a cycle owner");
    struct Release {std::atomic<bool>& running;~Release(){running.store(false);}} release{executing_};
    return step_impl(robot,now);
}

RuntimeSnapshot ControlRuntime::step_impl(Robot& robot, Robot::TimePoint now) {
    RuntimeSnapshot result;
    result.monotonic_ns=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
    if(robot.get_state()==RobotState::ACTIVE) {
        std::optional<PendingReference> pending;
        {std::lock_guard<std::mutex> lock(command_mutex_);pending=std::move(pending_);pending_.reset();result.source=source_;}
        if(pending) {
            auto sent=robot.set_cmd(pending->command,pending->received_at);
            if(!sent)result.fault=sent.error();
        }
        if(!result.fault) {
            auto cycle=robot.cycle(now);
            if(cycle)result.cycle=std::move(cycle.value());
            else result.fault=cycle.error();
        }
    }
    if(robot.get_state()==RobotState::FAULT) {
        clear_commands();
        if(robot.is_fault_holding()) {
            auto held=robot.maintain_fault_hold();
            result.fault_hold_maintained=bool(held);
            if(!held)result.fault=held.error();
        }
        if(!result.fault && robot.get_last_fault())result.fault=*robot.get_last_fault();
    }
    result.state=robot.get_state();
    if(result.state!=RobotState::ACTIVE){clear_commands();drag_suspension_.reset();result.source=MotionSource::NONE;}
    {std::lock_guard<std::mutex> lock(snapshot_mutex_);result.sequence=snapshot_.sequence+1;snapshot_=result;}
    return result;
}

void ControlRuntime::run(Robot& robot, double frequency_hz, double max_dt_s, const RuntimeHooks& hooks) {
    if(!std::isfinite(frequency_hz)||frequency_hz<=0||!hooks.running)
        throw std::invalid_argument("invalid control runtime configuration");
    if(executing_.exchange(true))throw std::logic_error("control runtime already has a periodic executor");
    struct Release {std::atomic<bool>& running;~Release(){running.store(false);}} release{executing_};
    const double nominal=1.0/frequency_hz;
    const auto period=std::chrono::duration_cast<Robot::Clock::duration>(std::chrono::duration<double>(nominal));
    auto next=Robot::Clock::now(),previous=next;
    bool first=true;
    while(hooks.running()) {
        std::unique_lock<std::mutex> lock;
        if(hooks.serialization_mutex)lock=std::unique_lock<std::mutex>(*hooks.serialization_mutex);
        auto now=Robot::Clock::now();
        if(now>next)next=now;
        double dt=first?nominal:std::chrono::duration<double>(now-previous).count();
        first=false;previous=now;
        if(!std::isfinite(dt)||dt<=0||dt>max_dt_s)dt=nominal;
        if(robot.get_state()!=RobotState::ACTIVE || !hooks.before_cycle || hooks.before_cycle(now,dt)) {
            auto result=step_impl(robot,now);
            if(hooks.after_step)hooks.after_step(result);
        }
        if(lock.owns_lock())lock.unlock();
        next+=period;
        std::this_thread::sleep_until(next);
    }
}

bool advance_joint_reference(JointVector& position, JointVector& velocity, const JointVector& target,
    const JointVector& max_velocity,const JointVector& max_acceleration,double speed_scale,double dt,
    const JointVector* measured) {
    const auto n=target.size();
    if(!finite(position,n)||!finite(velocity,n)||!finite(target,n)||!finite(max_velocity,n)||
       !finite(max_acceleration,n)||!std::isfinite(speed_scale)||speed_scale<0||speed_scale>1||
       !std::isfinite(dt)||dt<=0||(measured&&!finite(*measured,n)))
        throw std::invalid_argument("invalid joint reference input");
    bool complete=true;
    for(std::size_t i=0;i<n;++i) {
        if(max_velocity[i]<=0||max_acceleration[i]<=0)throw std::invalid_argument("invalid joint reference limits");
        const double error=target[i]-position[i];
        const double follow=measured?park_reference_speed_fraction(position[i],(*measured)[i]):1.0;
        const double vmax=max_velocity[i]*speed_scale*follow, dv=max_acceleration[i]*dt;
        // A discrete stopping envelope reserves one complete execution step
        // Position and velocity always describe the same backward-Euler step
        const double distance=std::abs(error), braking=max_acceleration[i]*.5;
        const double root=std::sqrt(2.0*braking*distance+.25*braking*braking*dt*dt);
        const double stopping_speed=distance==0?0.0:(2.0*braking*distance)/(root+.5*braking*dt);
        const double desired=(error>0?1.0:error<0?-1.0:0.0)*std::min(vmax,stopping_speed);
        const double response=std::max(.03,4.0*dt);
        const double acceleration=max_acceleration[i]*std::tanh((desired-velocity[i])/(max_acceleration[i]*response));
        double next_velocity=velocity[i]+std::clamp(acceleration*dt,-dv,dv);
        // A paused reference must reach exact rest in finitely many cycles;
        // the smooth tracking response alone only approaches zero asymptotically
        if(vmax==0.0)next_velocity=velocity[i]+std::clamp(-velocity[i],-dv,dv);
        const double capture=error/dt;
        // Final capture is allowed only when the matching velocity and its
        // acceleration are feasible, never by snapping q independently of dq
        if(std::abs(error)<=1e-5 && std::abs(capture)<=vmax && std::abs(capture-velocity[i])<=dv)
            next_velocity=capture;
        position[i]+=next_velocity*dt;
        velocity[i]=next_velocity;
        if(std::abs(target[i]-position[i])>1e-12||std::abs(velocity[i])>1e-12)complete=false;
    }
    return complete;
}
} // namespace serial_arm
