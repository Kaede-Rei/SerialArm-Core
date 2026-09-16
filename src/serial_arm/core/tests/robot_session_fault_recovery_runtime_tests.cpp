#include "robot_session.hpp"

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <thread>

namespace {

using namespace serial_arm;
using namespace std::chrono_literals;

template<typename Predicate>
bool wait_until(Predicate&& predicate, std::chrono::milliseconds timeout) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while(std::chrono::steady_clock::now() < deadline) {
        if(predicate()) return true;
        std::this_thread::sleep_for(5ms);
    }
    return predicate();
}

TEST(RobotSessionFaultRecoveryRuntime, RecoverableReadFaultKeepsWorkerAliveAndDropsStaleGoal) {
    PyRobotSession session;
    session.configure(
        SERIAL_ARM_TEST_DM_ARM_CORE_CONFIG,
        SERIAL_ARM_TEST_FAULT_INJECT_HARDWARE_PLUGIN,
        SERIAL_ARM_TEST_DM_ARM_HARDWARE_CONFIG);

    session.start();
    ASSERT_TRUE(session.is_running());
    ASSERT_EQ(session.get_state(), RobotState::ACTIVE);

    session.set_impedance_mode(JointImpedanceMode::RIGID_TRACKING);
    JointVector stale_goal(6, 0.0);
    stale_goal[0] = 0.8;
    session.move_to(stale_goal, 0.2);

    ASSERT_TRUE(wait_until([&] {
        return session.get_state() == RobotState::FAULT;
    }, 1500ms));
    EXPECT_TRUE(session.is_running());
    EXPECT_EQ(session.get_fault_hold_mode(), FaultHoldMode::RIGID_HOLD);

    // The worker must continue refreshing the hold long enough for the Core's
    // three-valid-cycle clear gate to become eligible.
    std::this_thread::sleep_for(50ms);

    bool cleared = false;
    for(int attempt = 0; attempt < 20 && !cleared; ++attempt) {
        try {
            session.clear_fault();
            cleared = true;
        }
        catch(const SerialArmPythonError&) {
            std::this_thread::sleep_for(10ms);
        }
    }
    ASSERT_TRUE(cleared);
    ASSERT_TRUE(session.is_running());
    ASSERT_EQ(session.get_state(), RobotState::ACTIVE);

    ASSERT_TRUE(wait_until([&] {
        const auto snapshot = session.get_snapshot();
        return snapshot.valid && snapshot.robot_state == RobotState::ACTIVE;
    }, 1000ms));

    const auto recovered_snapshot = session.get_snapshot();
    ASSERT_FALSE(recovered_snapshot.cycle.joint_state.pos.empty());
    const double recovered_pos = recovered_snapshot.cycle.joint_state.pos[0];
    EXPECT_LT(recovered_pos, stale_goal[0] - 0.05);

    // Re-enter tracking without submitting a new move_to(). If the pre-FAULT
    // goal survived, the fake backend would immediately continue toward 0.8.
    session.set_impedance_mode(JointImpedanceMode::RIGID_TRACKING);
    std::this_thread::sleep_for(120ms);

    const auto after_reentry = session.get_snapshot();
    ASSERT_TRUE(after_reentry.valid);
    ASSERT_FALSE(after_reentry.cycle.joint_state.pos.empty());
    EXPECT_NEAR(after_reentry.cycle.joint_state.pos[0], recovered_pos, 1e-6);
    EXPECT_TRUE(session.is_running());

    session.stop();
    EXPECT_FALSE(session.is_running());
    EXPECT_EQ(session.get_state(), RobotState::INACTIVE);
}

} // namespace
