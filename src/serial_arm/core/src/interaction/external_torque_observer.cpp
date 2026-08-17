#include "serial_arm/interaction/external_torque_observer.hpp"

#include <algorithm>
#include <cmath>

namespace serial_arm {

// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

namespace {

/**
 * @brief 检查 source 是否为有效枚举值
 * @param source residual source
 * @return 有效时返回 true，否则返回 false
 */
bool valid_source(ExternalTorqueSource source) {
    return source == ExternalTorqueSource::GRAVITY || source == ExternalTorqueSource::NONLINEAR;
}

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
 * @brief 验证关节向量的大小和有限性
 * @param values 关节向量
 * @param expected_size 期望大小
 * @return 成功时返回空值；失败时返回 ExternalTorqueObserverErr
 */
tl::expected<void, ExternalTorqueObserverErr> validate_joint_vector(const JointVector& values, std::size_t expected_size) {
    if(values.size() != expected_size) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_INPUT_SIZE);
    }
    if(!finite_vector(values)) {
        return tl::make_unexpected(ExternalTorqueObserverErr::NON_FINITE_INPUT);
    }
    return {};
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置 observer
 * @param cfg observer 配置
 * @return 成功时返回空值；失败时返回 ExternalTorqueObserverErr
 */
tl::expected<void, ExternalTorqueObserverErr> ExternalTorqueObserver::configure(const ExternalTorqueObserverCfg& cfg) {
    if(is_configured_) {
        return tl::make_unexpected(ExternalTorqueObserverErr::ALREADY_CONFIGURED);
    }
    if(cfg.joints_count == 0 || !valid_source(cfg.source)) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    }
    if(!cfg.residual_bias.empty() &&
        (cfg.residual_bias.size() != cfg.joints_count || !finite_vector(cfg.residual_bias))) {
        return tl::make_unexpected(ExternalTorqueObserverErr::INVALID_CFG);
    }

    cfg_ = cfg;
    if(cfg_.residual_bias.empty()) {
        cfg_.residual_bias.assign(cfg_.joints_count, 0.0);
    }
    is_configured_ = true;
    return {};
}

/**
 * @brief 使用 residual 估计外力矩
 * @param residual residual observer 输出
 * @return 成功时返回外力矩估计；失败时返回 ExternalTorqueObserverErr
 */
tl::expected<ExternalTorqueEstimate, ExternalTorqueObserverErr> ExternalTorqueObserver::update(const TorqueResidualEstimate& residual) const {
    if(!is_configured_) {
        return tl::make_unexpected(ExternalTorqueObserverErr::NOT_CONFIGURED);
    }

    const JointVector& source = cfg_.source == ExternalTorqueSource::GRAVITY ?
        residual.gravity_residual_filtered :
        residual.nonlinear_residual_filtered;
    const auto valid = validate_joint_vector(source, cfg_.joints_count);
    if(!valid) return tl::make_unexpected(valid.error());

    ExternalTorqueEstimate estimate;
    estimate.tau_ext_hat.resize(cfg_.joints_count);
    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        estimate.tau_ext_hat[i] = source[i] - cfg_.residual_bias[i];
        if(!std::isfinite(estimate.tau_ext_hat[i])) {
            return tl::make_unexpected(ExternalTorqueObserverErr::NON_FINITE_INPUT);
        }
    }
    return estimate;
}

/**
 * @brief 查询 observer 是否已经完成配置
 */
bool ExternalTorqueObserver::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
