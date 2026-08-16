#include "serial_arm/interaction/torque_residual_observer.hpp"

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
 * @brief 验证关节向量的大小和有限性
 * @param values 关节向量
 * @param expected_size 期望大小
 * @return 成功时返回空值；失败时返回 TorqueResidualObserverErr
 */
tl::expected<void, TorqueResidualObserverErr> validate_joint_vector(const JointVector& values, std::size_t expected_size) {
    if(values.size() != expected_size) {
        return tl::make_unexpected(TorqueResidualObserverErr::INVALID_INPUT_SIZE);
    }
    if(!finite_vector(values)) {
        return tl::make_unexpected(TorqueResidualObserverErr::NON_FINITE_INPUT);
    }
    return {};
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 配置 observer
 * @param cfg observer 配置
 * @return 成功时返回空值；失败时返回 TorqueResidualObserverErr
 */
tl::expected<void, TorqueResidualObserverErr> TorqueResidualObserver::configure(const TorqueResidualObserverCfg& cfg) {
    if(is_configured_) {
        return tl::make_unexpected(TorqueResidualObserverErr::ALREADY_CONFIGURED);
    }
    if(cfg.joints_count == 0 || !std::isfinite(cfg.filter_alpha) || cfg.filter_alpha <= 0.0 || cfg.filter_alpha > 1.0) {
        return tl::make_unexpected(TorqueResidualObserverErr::INVALID_CFG);
    }

    cfg_ = cfg;
    gravity_filtered_.assign(cfg_.joints_count, 0.0);
    nonlinear_filtered_.assign(cfg_.joints_count, 0.0);
    has_sample_ = false;
    is_configured_ = true;
    return {};
}

/**
 * @brief 使用当前 DynamicsState 计算 residual
 * @param state 动力学状态缓存
 * @return 成功时返回 residual 估计；失败时返回 TorqueResidualObserverErr
 */
tl::expected<TorqueResidualEstimate, TorqueResidualObserverErr> TorqueResidualObserver::update(const DynamicsState& state) {
    if(!is_configured_) {
        return tl::make_unexpected(TorqueResidualObserverErr::NOT_CONFIGURED);
    }

    const auto tor_valid = validate_joint_vector(state.tor, cfg_.joints_count);
    const auto gravity_valid = validate_joint_vector(state.gravity, cfg_.joints_count);
    const auto nonlinear_valid = validate_joint_vector(state.nonlinear, cfg_.joints_count);
    if(!tor_valid) return tl::make_unexpected(tor_valid.error());
    if(!gravity_valid) return tl::make_unexpected(gravity_valid.error());
    if(!nonlinear_valid) return tl::make_unexpected(nonlinear_valid.error());

    TorqueResidualEstimate estimate;
    estimate.gravity_residual.assign(cfg_.joints_count, 0.0);
    estimate.nonlinear_residual.assign(cfg_.joints_count, 0.0);
    estimate.gravity_residual_filtered.assign(cfg_.joints_count, 0.0);
    estimate.nonlinear_residual_filtered.assign(cfg_.joints_count, 0.0);

    for(std::size_t i = 0; i < cfg_.joints_count; ++i) {
        estimate.gravity_residual[i] = state.gravity[i] - state.tor[i];
        estimate.nonlinear_residual[i] = state.nonlinear[i] - state.tor[i];

        if(!has_sample_) {
            gravity_filtered_[i] = estimate.gravity_residual[i];
            nonlinear_filtered_[i] = estimate.nonlinear_residual[i];
        }
        else {
            gravity_filtered_[i] = cfg_.filter_alpha * estimate.gravity_residual[i] +
                (1.0 - cfg_.filter_alpha) * gravity_filtered_[i];
            nonlinear_filtered_[i] = cfg_.filter_alpha * estimate.nonlinear_residual[i] +
                (1.0 - cfg_.filter_alpha) * nonlinear_filtered_[i];
        }

        estimate.gravity_residual_filtered[i] = gravity_filtered_[i];
        estimate.nonlinear_residual_filtered[i] = nonlinear_filtered_[i];
    }

    has_sample_ = true;
    return estimate;
}

/**
 * @brief 清除滤波历史
 */
void TorqueResidualObserver::reset() {
    std::fill(gravity_filtered_.begin(), gravity_filtered_.end(), 0.0);
    std::fill(nonlinear_filtered_.begin(), nonlinear_filtered_.end(), 0.0);
    has_sample_ = false;
}

/**
 * @brief 查询 observer 是否已经完成配置
 */
bool TorqueResidualObserver::is_configured() const noexcept {
    return is_configured_;
}

} // namespace serial_arm
