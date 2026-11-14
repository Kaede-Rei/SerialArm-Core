#include "serial_arm/core/safety.hpp"
#include <cassert>
using namespace serial_arm;
int main(){
 SafetyCfg c; c.joints_count=1; auto& l=c.limits;
 l.has_position_limit={1};l.min_pos={-2};l.max_pos={2};l.max_vel={1};l.max_acc={2};l.max_effort={4};l.max_kp={500};l.max_kd={5};l.pos_margin={0};l.max_tracking_error={0.2};
 c.fault_recovery.compliant_recovery.kp={0};c.fault_recovery.compliant_recovery.kd={0};c.fault_recovery.compliant_recovery.max_vel={1};c.require_continuous_cmd=false;
 Safety s;assert(s.configure(c)); JointState state;state.pos={0};state.vel={0};state.tor={0};
 JointCtrlCmd cmd;cmd.path=ControlPath::CSP;cmd.pos={0.005};cmd.vel={0};cmd.tor={0};cmd.kp={0};cmd.kd={0};cmd.position_speed_limit={0.5};
 assert(s.check_joint_cmd(state,cmd,0.01));
 cmd.pos={0.4};auto e=s.check_joint_cmd(state,cmd,0.01);assert(!e && e.error().code==SafetyErr::CMD_TRACKING_ERROR);
 cmd.pos={0.005};state.tor={4.5};e=s.check_joint_cmd(state,cmd,0.01);assert(!e && e.error().code==SafetyErr::JOINT_EFFORT_LIMIT);assert(s.action_for(e.error().code)==SafetyAction::DISABLE);
 state.tor={0};cmd.pos={0.1};e=s.check_joint_cmd(state,cmd,0.01);assert(!e && e.error().code==SafetyErr::CMD_POS_STEP_LIMIT);
 cmd.pos={0.005};cmd.vel={0.5};e=s.check_joint_cmd(state,cmd,0.01);assert(!e && e.error().code==SafetyErr::CMD_VEL_STEP_LIMIT);
 c.limits.max_tracking_error={0};assert(!s.configure(c));
 c.limits.max_tracking_error={-0.1};assert(!s.configure(c));
 c.limits.max_tracking_error.clear();assert(s.configure(c));cmd.vel={0};e=s.check_joint_cmd(state,cmd,0.01);assert(!e && e.error().code==SafetyErr::INVALID_CFG);
}
