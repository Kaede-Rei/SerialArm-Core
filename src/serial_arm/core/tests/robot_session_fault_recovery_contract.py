#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
HPP = (ROOT / "python/robot_session.hpp").read_text(encoding="utf-8")
CPP = (ROOT / "python/robot_session.cpp").read_text(encoding="utf-8")


def body(signature: str) -> str:
    start = CPP.index(signature)
    brace = CPP.index("{", start)
    depth = 0
    for i in range(brace, len(CPP)):
        if CPP[i] == "{":
            depth += 1
        elif CPP[i] == "}":
            depth -= 1
            if depth == 0:
                return CPP[brace + 1:i]
    raise AssertionError(f"unterminated function: {signature}")


assert "#include <condition_variable>" in HPP
assert "enum class FaultRecoveryRequest" in HPP
assert "submit_fault_recovery_request" in HPP
assert "std::condition_variable" in HPP

loop = body("void PyRobotSession::loop() noexcept")
assert "robot_->is_fault_holding()" in loop
assert "robot_->maintain_fault_hold()" in loop
assert "process_fault_recovery_request" in loop

clear_fault = body("void PyRobotSession::clear_fault()")
assert "submit_fault_recovery_request(FaultRecoveryRequest::CLEAR_FAULT)" in clear_fault
assert "running_.store(false)" not in clear_fault
assert "worker_.join()" not in clear_fault

enter_compliant = body("void PyRobotSession::enter_fault_compliant_recovery()")
return_rigid = body("void PyRobotSession::return_to_fault_rigid_hold()")
assert "submit_fault_recovery_request(FaultRecoveryRequest::ENTER_COMPLIANT)" in enter_compliant
assert "submit_fault_recovery_request(FaultRecoveryRequest::RETURN_RIGID)" in return_rigid

start = body("void PyRobotSession::start()")
assert "robot_->is_fault_holding()" in start
assert "robot_->force_deactivate()" in start

stop = body("void PyRobotSession::stop()")
assert "RobotState::FAULT" in stop
assert "force_deactivate()" in stop

loop_tail = loop[loop.index("// A stopped worker"):]
assert "RobotState::FAULT" in loop_tail
assert "robot_->force_deactivate()" in loop_tail

for signature in (
    "void PyRobotSession::set_impedance_mode(JointImpedanceMode mode)",
    "void PyRobotSession::move_to(const JointVector& pos, double speed_scale)",
    "void PyRobotSession::hold_current()",
):
    fn = body(signature)
    assert "RobotState::ACTIVE" in fn, signature

assert "invalidate_motion_requests_for_fault" in HPP
invalidate = body("void PyRobotSession::invalidate_motion_requests_for_fault")
assert "has_goal_ = false" in invalidate
assert "JointImpedanceMode::RIGID_HOLD" in invalidate
assert "requested_gravity_scale_ = dynamics_->get_gravity_scale()" in invalidate

print("ROBOT_SESSION_FAULT_RECOVERY_CONTRACT_PASS")
