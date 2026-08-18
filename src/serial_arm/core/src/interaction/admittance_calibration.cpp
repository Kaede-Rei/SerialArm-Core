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

double estimate_repeated_torque_quantization_step(
    const std::vector<AdmittanceStaticPoseSamples>& poses,
    std::size_t joint) {
    constexpr std::size_t kMinSamplesPerLevel = 3;
    constexpr double kLevelEqualityTolerance = 1e-9;

    double estimated_step = std::numeric_limits<double>::infinity();
    for(const auto& pose : poses) {
        if(pose.samples.empty()) continue;

        std::vector<double> values;
        values.reserve(pose.samples.size());
        for(const auto& sample : pose.samples) {
            values.push_back(sample.measured_torque[joint]);
        }
        std::sort(values.begin(), values.end());

        struct RepeatedLevel {
            double value{ 0.0 };
            std::size_t count{ 0 };
        };
        std::vector<RepeatedLevel> repeated_levels;

        std::size_t begin = 0;
        while(begin < values.size()) {
            std::size_t end = begin + 1;
            const double scale = std::max(1.0, std::abs(values[begin]));
            const double tolerance = kLevelEqualityTolerance * scale;
            while(end < values.size() && std::abs(values[end] - values[begin]) <= tolerance) {
                ++end;
            }
            const std::size_t count = end - begin;
            if(count >= kMinSamplesPerLevel) {
                repeated_levels.push_back(RepeatedLevel{ values[begin], count });
            }
            begin = end;
        }

        for(std::size_t i = 1; i < repeated_levels.size(); ++i) {
            const double step = repeated_levels[i].value - repeated_levels[i - 1].value;
            if(step > kLevelEqualityTolerance) {
                estimated_step = std::min(estimated_step, step);
            }
        }
    }

    return std::isfinite(estimated_step) ? estimated_step : 0.0;
}

struct StaticJointFit {
    double gravity_scale{ 1.0 };
    double torque_bias{ 0.0 };
    std::uint8_t gravity_scale_observable{ 0 };
};

StaticJointFit fit_static_joint(
    const std::vector<double>& pose_gravity,
    const std::vector<double>& pose_torque,
    double fallback_gravity_scale,
    double gravity_observability_span) {
    StaticJointFit fit;
    fit.gravity_scale = fallback_gravity_scale;
    if(pose_gravity.empty()) return fit;

    const auto [min_it, max_it] = std::minmax_element(pose_gravity.begin(), pose_gravity.end());
    const double gravity_span = *max_it - *min_it;
    if(gravity_span >= gravity_observability_span) {
        const double mean_g = std::accumulate(pose_gravity.begin(), pose_gravity.end(), 0.0) /
            static_cast<double>(pose_gravity.size());
        const double mean_t = std::accumulate(pose_torque.begin(), pose_torque.end(), 0.0) /
            static_cast<double>(pose_torque.size());
        double covariance = 0.0;
        double variance = 0.0;
        for(std::size_t i = 0; i < pose_gravity.size(); ++i) {
            const double dg = pose_gravity[i] - mean_g;
            covariance += dg * (pose_torque[i] - mean_t);
            variance += dg * dg;
        }
        if(variance > std::numeric_limits<double>::epsilon()) {
            fit.gravity_scale = std::clamp(covariance / variance, 0.0, 1.0);
            fit.gravity_scale_observable = 1;
        }
    }

    std::vector<double> residual_base;
    residual_base.reserve(pose_gravity.size());
    for(std::size_t i = 0; i < pose_gravity.size(); ++i) {
        residual_base.push_back(fit.gravity_scale * pose_gravity[i] - pose_torque[i]);
    }
    fit.torque_bias = median(std::move(residual_base));
    return fit;
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
    result.within_pose_residual_p99.assign(cfg.joints_count, 0.0);
    result.within_pose_residual_max.assign(cfg.joints_count, 0.0);
    result.lopo_residual_p99.assign(cfg.joints_count, 0.0);
    result.lopo_residual_max.assign(cfg.joints_count, 0.0);
    result.residual_p99.assign(cfg.joints_count, 0.0);
    result.residual_max.assign(cfg.joints_count, 0.0);
    result.gravity_scale_observable.assign(cfg.joints_count, 0);

    for(std::size_t joint = 0; joint < cfg.joints_count; ++joint) {
        // gravity_scale / torque_bias describe the robot itself, so each manually selected
        // static pose must contribute equal weight regardless of scheduler/sample-count jitter.
        // Reduce every pose to a robust median first, then fit across pose representatives.
        std::vector<double> pose_gravity;
        std::vector<double> pose_torque;
        pose_gravity.reserve(poses.size());
        pose_torque.reserve(poses.size());
        for(const auto& pose : poses) {
            if(pose.samples.empty()) continue;
            std::vector<double> gravity_values;
            std::vector<double> torque_values;
            gravity_values.reserve(pose.samples.size());
            torque_values.reserve(pose.samples.size());
            for(const auto& sample : pose.samples) {
                gravity_values.push_back(sample.gravity[joint]);
                torque_values.push_back(sample.measured_torque[joint]);
            }
            pose_gravity.push_back(median(std::move(gravity_values)));
            pose_torque.push_back(median(std::move(torque_values)));
        }

        const auto full_fit = fit_static_joint(
            pose_gravity,
            pose_torque,
            cfg.fallback_gravity_scale[joint],
            cfg.gravity_observability_span);
        result.gravity_scale[joint] = full_fit.gravity_scale;
        result.torque_bias[joint] = full_fit.torque_bias;
        result.gravity_scale_observable[joint] = full_fit.gravity_scale_observable;

        // Envelope A: within-pose static noise. Remove each pose's own robust center so
        // cross-pose model mismatch does not get confused with sensor/static-friction jitter.
        std::vector<double> within_pose_abs_error;
        within_pose_abs_error.reserve(total_samples);
        for(const auto& pose : poses) {
            if(pose.samples.empty()) continue;
            std::vector<double> pose_raw_residual;
            pose_raw_residual.reserve(pose.samples.size());
            for(const auto& sample : pose.samples) {
                pose_raw_residual.push_back(
                    full_fit.gravity_scale * sample.gravity[joint] - sample.measured_torque[joint]);
            }
            const double pose_center = median(pose_raw_residual);
            for(const double raw : pose_raw_residual) {
                within_pose_abs_error.push_back(std::abs(raw - pose_center));
            }
        }

        // Envelope B: Leave-One-Pose-Out generalization error. Every manually selected
        // pose must once be predicted by a gravity_scale / bias fit that did not see it.
        // This prevents torque_threshold from being estimated by the same pose data used
        // to fit the static model itself.
        std::vector<double> lopo_abs_error;
        if(pose_gravity.size() >= 2) {
            for(std::size_t held_out = 0; held_out < pose_gravity.size(); ++held_out) {
                std::vector<double> train_gravity;
                std::vector<double> train_torque;
                train_gravity.reserve(pose_gravity.size() - 1);
                train_torque.reserve(pose_torque.size() - 1);
                for(std::size_t i = 0; i < pose_gravity.size(); ++i) {
                    if(i == held_out) continue;
                    train_gravity.push_back(pose_gravity[i]);
                    train_torque.push_back(pose_torque[i]);
                }
                const auto fold_fit = fit_static_joint(
                    train_gravity,
                    train_torque,
                    cfg.fallback_gravity_scale[joint],
                    cfg.gravity_observability_span);

                std::size_t representative_index = 0;
                for(const auto& pose : poses) {
                    if(pose.samples.empty()) continue;
                    if(representative_index == held_out) {
                        for(const auto& sample : pose.samples) {
                            const double residual = fold_fit.gravity_scale * sample.gravity[joint] -
                                sample.measured_torque[joint] - fold_fit.torque_bias;
                            lopo_abs_error.push_back(std::abs(residual));
                        }
                        break;
                    }
                    ++representative_index;
                }
            }
        }

        result.within_pose_residual_p99[joint] = percentile(within_pose_abs_error, 0.99);
        result.within_pose_residual_max[joint] = within_pose_abs_error.empty() ? 0.0 :
            *std::max_element(within_pose_abs_error.begin(), within_pose_abs_error.end());
        result.lopo_residual_p99[joint] = percentile(lopo_abs_error, 0.99);
        result.lopo_residual_max[joint] = lopo_abs_error.empty() ? 0.0 :
            *std::max_element(lopo_abs_error.begin(), lopo_abs_error.end());

        result.residual_p99[joint] = std::max(
            result.within_pose_residual_p99[joint],
            result.lopo_residual_p99[joint]);
        result.residual_max[joint] = std::max(
            result.within_pose_residual_max[joint],
            result.lopo_residual_max[joint]);
        const double envelope_threshold = std::max(
            cfg.threshold_margin * result.residual_p99[joint],
            cfg.threshold_max_margin * result.residual_max[joint]);

        // Quantized torque feedback can make an otherwise valid static sample land one
        // discrete code beyond the largest code observed during calibration. Infer the
        // smallest repeatedly occupied feedback spacing from the raw static samples and
        // reserve exactly one additional step beyond the calibrated residual envelope.
        // Requiring >=3 samples at both levels prevents a single isolated outlier from
        // being misclassified as feedback quantization.
        const double torque_quantization_step = estimate_repeated_torque_quantization_step(poses, joint);
        double quantization_guard = 0.0;
        if(torque_quantization_step > 0.0) {
            constexpr double kNumericGuardMargin = 1e-6;
            quantization_guard = (result.residual_max[joint] + torque_quantization_step) *
                (1.0 + kNumericGuardMargin);
        }

        result.torque_threshold[joint] = std::max(envelope_threshold, quantization_guard);
    }

    return result;
}


tl::expected<AdmittanceStaticValidationResult, AdmittanceStaticCalibrationErr>
evaluate_admittance_static_validation(
    const std::vector<AdmittanceStaticPoseSamples>& poses,
    const AdmittanceStaticValidationCfg& cfg) {
    if(cfg.joints_count == 0 ||
        cfg.gravity_scale.size() != cfg.joints_count ||
        cfg.torque_bias.size() != cfg.joints_count ||
        cfg.torque_threshold.size() != cfg.joints_count ||
        !finite_vector(cfg.gravity_scale) ||
        !finite_vector(cfg.torque_bias) ||
        !finite_vector(cfg.torque_threshold) ||
        std::any_of(cfg.torque_threshold.begin(), cfg.torque_threshold.end(), [](double value) { return value < 0.0; })) {
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

    AdmittanceStaticValidationResult result;
    result.gravity_span.assign(cfg.joints_count, 0.0);
    result.residual_rms.assign(cfg.joints_count, 0.0);
    result.residual_p99.assign(cfg.joints_count, 0.0);
    result.residual_max.assign(cfg.joints_count, 0.0);
    result.feedback_quantization_step.assign(cfg.joints_count, 0.0);
    result.guarded_max_limit.assign(cfg.joints_count, 0.0);
    result.threshold_utilization.assign(cfg.joints_count, 0.0);
    result.guarded_max_utilization.assign(cfg.joints_count, 0.0);
    result.pass.assign(cfg.joints_count, 0);

    for(std::size_t joint = 0; joint < cfg.joints_count; ++joint) {
        std::vector<double> gravity_values;
        std::vector<double> abs_residuals;
        gravity_values.reserve(total_samples);
        abs_residuals.reserve(total_samples);
        double sum_sq = 0.0;

        for(const auto& pose : poses) {
            for(const auto& sample : pose.samples) {
                const double gravity = sample.gravity[joint];
                const double residual = cfg.gravity_scale[joint] * gravity -
                    sample.measured_torque[joint] - cfg.torque_bias[joint];
                gravity_values.push_back(gravity);
                abs_residuals.push_back(std::abs(residual));
                sum_sq += residual * residual;
            }
        }

        const auto [min_it, max_it] = std::minmax_element(gravity_values.begin(), gravity_values.end());
        result.gravity_span[joint] = *max_it - *min_it;
        result.residual_rms[joint] = std::sqrt(sum_sq / static_cast<double>(abs_residuals.size()));
        result.residual_p99[joint] = percentile(abs_residuals, 0.99);
        result.residual_max[joint] = *std::max_element(abs_residuals.begin(), abs_residuals.end());

        const double threshold = cfg.torque_threshold[joint];
        const double quantization_step = estimate_repeated_torque_quantization_step(poses, joint);
        constexpr double kNumericGuardMargin = 1e-6;
        const double guarded_max_limit = threshold +
            quantization_step * (1.0 + kNumericGuardMargin);
        result.feedback_quantization_step[joint] = quantization_step;
        result.guarded_max_limit[joint] = guarded_max_limit;

        if(threshold > 0.0) {
            result.threshold_utilization[joint] = result.residual_p99[joint] / threshold;
            result.guarded_max_utilization[joint] = guarded_max_limit > 0.0 ?
                result.residual_max[joint] / guarded_max_limit : std::numeric_limits<double>::infinity();
            const bool p99_ok = result.residual_p99[joint] <= threshold * (1.0 + kNumericGuardMargin);
            const bool max_ok = result.residual_max[joint] <= guarded_max_limit;
            result.pass[joint] = (p99_ok && max_ok) ? 1 : 0;
        }
        else {
            result.threshold_utilization[joint] = result.residual_p99[joint] == 0.0 ?
                0.0 : std::numeric_limits<double>::infinity();
            result.guarded_max_utilization[joint] = guarded_max_limit > 0.0 ?
                result.residual_max[joint] / guarded_max_limit :
                (result.residual_max[joint] == 0.0 ? 0.0 : std::numeric_limits<double>::infinity());
            result.pass[joint] = (result.residual_p99[joint] == 0.0 &&
                result.residual_max[joint] <= guarded_max_limit) ? 1 : 0;
        }
    }

    return result;
}

} // namespace serial_arm
