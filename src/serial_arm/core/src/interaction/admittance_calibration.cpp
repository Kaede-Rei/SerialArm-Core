#include "serial_arm/interaction/admittance_calibration.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numeric>

namespace serial_arm {
namespace {

bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) {
        return std::isfinite(value);
    });
}

double percentile(std::vector<double> values, double quantile) {
    if(values.empty()) return 0.0;
    std::sort(values.begin(), values.end());
    const double position = quantile * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    if(lower == upper) return values[lower];
    const double ratio = position - static_cast<double>(lower);
    return values[lower] * (1.0 - ratio) + values[upper] * ratio;
}

double median(std::vector<double> values) {
    return percentile(std::move(values), 0.5);
}

} // namespace

tl::expected<AdmittanceStaticCalibrationResult, AdmittanceStaticCalibrationErr>
calibrate_admittance_static(
    const std::vector<AdmittanceStaticPoseSamples>& poses,
    const AdmittanceStaticCalibrationCfg& cfg) {
    if(cfg.joints_count == 0 ||
        cfg.fallback_gravity_scale.size() != cfg.joints_count ||
        !finite_vector(cfg.fallback_gravity_scale) ||
        !std::isfinite(cfg.gravity_observability_span) || cfg.gravity_observability_span < 0.0 ||
        !std::isfinite(cfg.threshold_margin) || cfg.threshold_margin < 1.0 ||
        !std::isfinite(cfg.threshold_max_margin) || cfg.threshold_max_margin < 1.0) {
        return tl::make_unexpected(AdmittanceStaticCalibrationErr::INVALID_CFG);
    }

    std::size_t total_samples = 0;
    for(const auto& pose : poses) {
        total_samples += pose.samples.size();
        for(const auto& sample : pose.samples) {
            if(sample.gravity.size() != cfg.joints_count || sample.measured_torque.size() != cfg.joints_count) {
                return tl::make_unexpected(AdmittanceStaticCalibrationErr::INVALID_SAMPLE_SIZE);
            }
            if(!finite_vector(sample.gravity) || !finite_vector(sample.measured_torque)) {
                return tl::make_unexpected(AdmittanceStaticCalibrationErr::NON_FINITE_SAMPLE);
            }
        }
    }
    if(total_samples == 0) {
        return tl::make_unexpected(AdmittanceStaticCalibrationErr::EMPTY_SAMPLES);
    }

    AdmittanceStaticCalibrationResult result;
    result.gravity_scale = cfg.fallback_gravity_scale;
    result.torque_bias.assign(cfg.joints_count, 0.0);
    result.torque_threshold.assign(cfg.joints_count, 0.0);
    result.residual_p99.assign(cfg.joints_count, 0.0);
    result.residual_max.assign(cfg.joints_count, 0.0);
    result.gravity_scale_observable.assign(cfg.joints_count, 0);

    for(std::size_t joint = 0; joint < cfg.joints_count; ++joint) {
        std::vector<double> gravity_values;
        std::vector<double> torque_values;
        gravity_values.reserve(total_samples);
        torque_values.reserve(total_samples);
        for(const auto& pose : poses) {
            for(const auto& sample : pose.samples) {
                gravity_values.push_back(sample.gravity[joint]);
                torque_values.push_back(sample.measured_torque[joint]);
            }
        }

        const auto [min_it, max_it] = std::minmax_element(gravity_values.begin(), gravity_values.end());
        const double gravity_span = *max_it - *min_it;
        if(gravity_span >= cfg.gravity_observability_span) {
            const double mean_g = std::accumulate(gravity_values.begin(), gravity_values.end(), 0.0) /
                static_cast<double>(gravity_values.size());
            const double mean_t = std::accumulate(torque_values.begin(), torque_values.end(), 0.0) /
                static_cast<double>(torque_values.size());
            double covariance = 0.0;
            double variance = 0.0;
            for(std::size_t i = 0; i < gravity_values.size(); ++i) {
                const double dg = gravity_values[i] - mean_g;
                covariance += dg * (torque_values[i] - mean_t);
                variance += dg * dg;
            }
            if(variance > std::numeric_limits<double>::epsilon()) {
                result.gravity_scale[joint] = std::clamp(covariance / variance, 0.0, 1.0);
                result.gravity_scale_observable[joint] = 1;
            }
        }

        std::vector<double> residual_base;
        residual_base.reserve(total_samples);
        for(std::size_t i = 0; i < gravity_values.size(); ++i) {
            residual_base.push_back(
                result.gravity_scale[joint] * gravity_values[i] - torque_values[i]);
        }
        result.torque_bias[joint] = median(residual_base);

        std::vector<double> abs_static_error;
        abs_static_error.reserve(total_samples);
        for(const auto& pose : poses) {
            for(const auto& sample : pose.samples) {
                const double raw = result.gravity_scale[joint] * sample.gravity[joint] - sample.measured_torque[joint];
                abs_static_error.push_back(std::abs(raw - result.torque_bias[joint]));
            }
        }

        result.residual_p99[joint] = percentile(abs_static_error, 0.99);
        result.residual_max[joint] = abs_static_error.empty() ? 0.0 :
            *std::max_element(abs_static_error.begin(), abs_static_error.end());
        result.torque_threshold[joint] = std::max(
            cfg.threshold_margin * result.residual_p99[joint],
            cfg.threshold_max_margin * result.residual_max[joint]);
    }

    return result;
}

} // namespace serial_arm
