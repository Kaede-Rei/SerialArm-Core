#include "serial_arm/core/joints_ctrller.hpp"
#include <cassert>
#include <iostream>

using namespace serial_arm;

int main() {
    JointCtrller controller;
    JointCtrllerCfg cfg;
    cfg.joints_count = 1;
    cfg.rigid_gains = {{10}, {2}};
    cfg.compliant_gains = {{3}, {1}};
    cfg.drag_gains = {{0}, {0.5}};
    auto invalid=cfg; invalid.drag_gains.kp[0]=5;
    assert(!controller.configure(invalid));
    assert(controller.configure(cfg));
    JointState state{{0.1}, {0}, {0}};
    assert(controller.initialize(state));
    assert(controller.set_cmd(JointPosVelCmd{{0.3}, {0.2}}));
    const JointCtrllerInput input{state, {0.4}, 0.005};
    for(const auto mode : {JointImpedanceMode::RIGID, JointImpedanceMode::COMPLIANT,
                           JointImpedanceMode::DRAG, JointImpedanceMode::SERVO_TRACKING}) {
        assert(controller.set_impedance_mode(mode, state));
        const auto out = controller.update(input);
        assert(out);
        assert(out->cmd.pos[0] == 0.3 && out->cmd.vel[0] == 0.2 && out->cmd.tor[0] == 0.4);
        const double kp = mode == JointImpedanceMode::DRAG ? 0 :
                          mode == JointImpedanceMode::COMPLIANT ? 3 : 10;
        const double kd = mode == JointImpedanceMode::DRAG ? 0.5 :
                          mode == JointImpedanceMode::COMPLIANT ? 1 : 2;
        assert(out->cmd.kp[0] == kp && out->cmd.kd[0] == kd);
        assert(controller.set_cmd(JointPosVelCmd{{0.3}, {0.2}}));
    }
    std::cout << "interaction modes passed\n";
}
