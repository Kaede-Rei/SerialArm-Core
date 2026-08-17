#include "serial_arm/interaction/joint_admittance_controller.hpp"

#include <algorithm>
#include <cmath>

namespace serial_arm {

// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 检查关节向量是否包含有限值
 * @param values 关节向量
 * @return 所有值均有限时返回 true，否则返回 false
 */
bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
        });
}

/**
 * @brief 检查关节向量大小和值域
 * @param values 关节向量
 * @param expected_size 期望大小
 * @param allow_zero 是否允许 0
 * @return 成功时返回 true，否则返回 false
 */
bool valid_cfg_vector(const JointVector& values, std::size_t expected_size, bool allow_zero) {
    if(values.size() != expected_size || !finite_vector(values)) return false;
    return std::all_of(values.begin(), values.end(), [allow_zero](double value) {
        return allow_zero ? value >= 0.0 : value > 0.0;
        });
}

/**
 * @brief 限幅
 * @param value 输入值
 * @param limit 对称限幅
 * @return 限幅后的值
 */
double clamp_symmetric(double value, double limit) {
    return std::clamp(value, -limit, limit);
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置 controller
 * @param cfg controller 配置
 * @return 成功时返回空值；失败时返回 JointAdmittanceControllerErr
 */
tl::expected<void, JointAdmittanceControllerErr> JointAdmittanceController::configure(const JointAdmittanceControllerCfg& cfg) {
    if(is_configured_) {
        return tl::make_unexpected(JointAdmittanceControllerErr::ALREADY_CONFIGURED);
    }
    if(cfg.joints_count == 0 ||
        cfg.enabled.size() != cfg.joints_count ||
        !valid_cfg_vector(cfg.mass, cfg.joints_count, false) ||
        !valid_cfg_vector(cfg.damping, cfg.joints_count, true) ||
        !valid_cfg_vector(cfg.stiffness, cfg.joints_count, true) ||
        !valid_cfg_vector(cfg.max_delta_q, cfg.joints_count, true) ||
        !valid_cfg_vector(cfg.max_delta_q_dot, cfg.joints_count, true)) {
        return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_CFG);
    }

    if(!std::all_of(cfg.enabled.begin(), cfg.enabled.end(), [](std::uint8_t value) {
        return value == 0 || value == 1;
        })) {
        return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_CFG);
    }

    cfg_ = cfg;
    delta_q_.assign(cfg_.joints_count, 0.0);
    delta_q_dot_.assign(cfg_.joints_count, 0.0);
    is_configured_ = true;
    return {};
}

/**
 * @brief 更新导纳状态
 * @param input 单周期输入
 * @return 成功时返回导纳输出；失败时返回 JointAdmittanceControllerErr
 */
tl::expected<JointAdmittanceOutput, JointAdmittanceControllerErr> JointAdmittanceController::update(const JointAdmittanceInput& input) {
    if(!is_configured_) {
        return tl::make_unexpected(JointAdmittanceControllerErr::NOT_CONFIGURED);
    }
    if(input.tau_ext_hat.size() != cfg_.joints_count) {
        return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_INPUT_SIZE);
    }
    if(!finite_vector(input.tau_ext_hat)) {
        return tl::make_unexpected(JointAdmittanceControllerErr::NON_FINITE_INPUT);
    }
    if(!std::isfinite(input.dt) || input.dt <= 0.0) {
        return tl::make_unexpected(JointAdmittanceControllerErr::INVALID_DT);
    }

    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        if(cfg_.enabled[i] == 0) {
            delta_q_[i] = 0.0;
            delta_q_dot_[i] = 0.0;
            continue;
        }

        const double delta_q_ddot = (input.tau_ext_hat[i] -
            cfg_.damping[i] * delta_q_dot_[i] -
            cfg_.stiffness[i] * delta_q_[i]) / cfg_.mass[i];
        delta_q_dot_[i] += delta_q_ddot * input.dt;
        delta_q_dot_[i] = clamp_symmetric(delta_q_dot_[i], cfg_.max_delta_q_dot[i]);
        delta_q_[i] += delta_q_dot_[i] * input.dt;
        delta_q_[i] = clamp_symmetric(delta_q_[i], cfg_.max_delta_q[i]);

        if(!std::isfinite(delta_q_[i]) || !std::isfinite(delta_q_dot_[i])) {
            return tl::make_unexpected(JointAdmittanceControllerErr::NON_FINITE_INPUT);
        }
    }

    return JointAdmittanceOutput{ delta_q_, delta_q_dot_ };
}

/**
 * @brief 清除导纳状态
 */
void JointAdmittanceController::reset() {
    std::fill(delta_q_.begin(), delta_q_.end(), 0.0);
    std::fill(delta_q_dot_.begin(), delta_q_dot_.end(), 0.0);
}

/**
 * @brief 查询 controller 是否已经完成配置
 */
bool JointAdmittanceController::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
