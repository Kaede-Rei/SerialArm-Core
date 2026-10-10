#include "serial_arm/dynamics/gravity_calibration.hpp"

#include <Eigen/SVD>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <numeric>
#include <set>
#include <limits>
#include <sstream>
#include <unordered_map>

namespace serial_arm {
namespace {

bool finite_vector(const JointVector& values) {
    return std::all_of(values.begin(), values.end(), [](double value) { return std::isfinite(value); });
}

std::string json_escape(const std::string& value) {
    std::ostringstream out;
    for(const unsigned char c : value) {
        switch(c) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default:
                if(c < 0x20) out << "\\u" << std::hex << std::setw(4) << std::setfill('0') << static_cast<int>(c) << std::dec << std::setfill(' ');
                else out << static_cast<char>(c);
        }
    }
    return out.str();
}

void json_vector(std::ostream& out, const JointVector& values) {
    out << '[';
    for(std::size_t i = 0; i < values.size(); ++i) {
        if(i) out << ',';
        out << std::setprecision(17) << values[i];
    }
    out << ']';
}

double percentile_abs(std::vector<double> values, double quantile) {
    if(values.empty()) return 0.0;
    for(double& value : values) value = std::abs(value);
    std::sort(values.begin(), values.end());
    const double position = quantile * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = std::min(lower + 1, values.size() - 1);
    const double ratio = position - static_cast<double>(lower);
    return values[lower] * (1.0 - ratio) + values[upper] * ratio;
}

GravityValidationMetrics metrics_from_residuals(
    const std::vector<JointVector>& residuals,
    const JointVector& bias) {
    GravityValidationMetrics result;
    if(residuals.empty()) return result;
    const std::size_t joints = residuals.front().size();
    result.rms.assign(joints, 0.0);
    result.p99.assign(joints, 0.0);
    result.maximum.assign(joints, 0.0);
    result.bias = bias;
    for(std::size_t joint = 0; joint < joints; ++joint) {
        std::vector<double> values;
        values.reserve(residuals.size());
        double sum_sq = 0.0;
        for(const auto& residual : residuals) {
            const double value = residual[joint] - bias[joint];
            values.push_back(value);
            sum_sq += value * value;
            result.maximum[joint] = std::max(result.maximum[joint], std::abs(value));
        }
        result.rms[joint] = std::sqrt(sum_sq / static_cast<double>(values.size()));
        result.p99[joint] = percentile_abs(std::move(values), 0.99);
    }
    return result;
}

JointVector estimate_bias(const std::vector<JointVector>& residuals) {
    if(residuals.empty()) return {};
    JointVector result(residuals.front().size(), 0.0);
    for(const auto& row : residuals) {
        for(std::size_t i = 0; i < result.size(); ++i) result[i] += row[i];
    }
    for(double& value : result) value /= static_cast<double>(residuals.size());
    return result;
}

struct PoseStatistic {
    std::size_t id{ 0 };
    bool validation{ false };
    JointVector q;
    JointVector tau;
    JointVector tau_std;
};

std::vector<PoseStatistic> pose_statistics(const std::vector<GravityCalibrationPoseGroup>& groups) {
    std::vector<PoseStatistic> result;
    for(const auto& group : groups) {
        if(group.samples.empty()) continue;
        const std::size_t joints = group.samples.front().position.size();
        PoseStatistic stat;
        stat.id = group.pose_group;
        stat.validation = group.validation;
        stat.q.assign(joints, 0.0);
        stat.tau.assign(joints, 0.0);
        stat.tau_std.assign(joints, 0.0);
        std::size_t valid = 0;
        for(const auto& sample : group.samples) {
            if(!sample.valid || sample.position.size() != joints || sample.torque.size() != joints ||
                !finite_vector(sample.position) || !finite_vector(sample.torque)) continue;
            ++valid;
            for(std::size_t joint = 0; joint < joints; ++joint) {
                stat.q[joint] += sample.position[joint];
                stat.tau[joint] += sample.torque[joint];
            }
        }
        if(valid == 0) continue;
        for(std::size_t joint = 0; joint < joints; ++joint) {
            stat.q[joint] /= static_cast<double>(valid);
            stat.tau[joint] /= static_cast<double>(valid);
        }
        for(const auto& sample : group.samples) {
            if(!sample.valid || sample.torque.size() != joints) continue;
            for(std::size_t joint = 0; joint < joints; ++joint) {
                const double error = sample.torque[joint] - stat.tau[joint];
                stat.tau_std[joint] += error * error;
            }
        }
        for(double& value : stat.tau_std) value = std::sqrt(value / static_cast<double>(valid));
        result.push_back(std::move(stat));
    }
    return result;
}

GravityValidationMetrics evaluate_model(
    Dynamics& dynamics,
    const std::vector<PoseStatistic>& poses,
    bool validation,
    const std::vector<GravityFirstMoment>* candidate,
    const JointVector* scale,
    const JointVector& bias) {
    std::vector<JointVector> residuals;
    for(const auto& pose : poses) {
        if(pose.validation != validation) continue;
        JointVector gravity;
        if(candidate) {
            const auto value = dynamics.compute_gravity_with_first_moments(pose.q, *candidate);
            if(!value) continue;
            gravity = value.value();
        } else {
            const auto regression = dynamics.get_gravity_regression(pose.q);
            if(!regression) continue;
            gravity = regression->original_gravity;
            if(scale && scale->size() == gravity.size()) {
                for(std::size_t i = 0; i < gravity.size(); ++i) gravity[i] *= (*scale)[i];
            }
        }
        JointVector residual(gravity.size(), 0.0);
        for(std::size_t i = 0; i < gravity.size(); ++i) residual[i] = gravity[i] - pose.tau[i];
        residuals.push_back(std::move(residual));
    }
    return metrics_from_residuals(residuals, bias);
}

JointVector estimate_model_bias(
    Dynamics& dynamics,
    const std::vector<PoseStatistic>& poses,
    const std::vector<GravityFirstMoment>* candidate,
    const JointVector* scale) {
    std::vector<JointVector> residuals;
    for(const auto& pose : poses) {
        if(pose.validation) continue;
        JointVector gravity;
        if(candidate) {
            const auto value = dynamics.compute_gravity_with_first_moments(pose.q, *candidate);
            if(!value) continue;
            gravity = value.value();
        } else {
            const auto regression = dynamics.get_gravity_regression(pose.q);
            if(!regression) continue;
            gravity = regression->original_gravity;
            if(scale && scale->size() == gravity.size()) for(std::size_t i = 0; i < gravity.size(); ++i) gravity[i] *= (*scale)[i];
        }
        JointVector residual(gravity.size(), 0.0);
        for(std::size_t i = 0; i < gravity.size(); ++i) residual[i] = gravity[i] - pose.tau[i];
        residuals.push_back(std::move(residual));
    }
    return estimate_bias(residuals);
}

} // namespace

ModelCalibrationRecorder::ModelCalibrationRecorder(std::size_t capacity) : capacity_(std::max<std::size_t>(capacity, 128)) {}

ModelCalibrationRecorder::~ModelCalibrationRecorder() { stop(); }

tl::expected<void, std::string> ModelCalibrationRecorder::start(
    const std::filesystem::path& directory,
    const ModelCalibrationMetadata& metadata,
    const std::vector<JointVector>& trajectory) {
    stop();
    std::error_code error;
    std::filesystem::create_directories(directory, error);
    if(error) return tl::make_unexpected("failed to create calibration directory: " + error.message());
    {
        std::lock_guard<std::mutex> lock(mutex_);
        metadata_ = metadata;
        directory_ = std::filesystem::absolute(directory);
        queue_.clear();
        frames_.clear();
        status_ = {};
        status_.directory = directory_.string();
        stop_requested_.store(false);
        running_.store(true);
    }
    const auto meta = write_metadata();
    if(!meta) {
        running_.store(false);
        return tl::make_unexpected(meta.error());
    }
    if(!trajectory.empty()) {
        const auto result = write_trajectory(trajectory, 0.0);
        if(!result) {
            running_.store(false);
            return tl::make_unexpected(result.error());
        }
    }
    writer_ = std::thread([this]() { writer_loop(); });
    return {};
}

void ModelCalibrationRecorder::push(ModelCalibrationFrame frame) {
    if(!running_.load()) return;
    std::lock_guard<std::mutex> lock(mutex_);
    ++status_.accepted_frames;
    if(queue_.size() >= capacity_) {
        ++status_.dropped_frames;
        status_.data_gap = true;
        return;
    }
    queue_.push_back(std::move(frame));
    cv_.notify_one();
}

void ModelCalibrationRecorder::stop() {
    stop_requested_.store(true);
    cv_.notify_all();
    if(writer_.joinable()) writer_.join();
    running_.store(false);
}

ModelCalibrationRecorderStatus ModelCalibrationRecorder::status() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return status_;
}

std::vector<ModelCalibrationFrame> ModelCalibrationRecorder::frames() const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<ModelCalibrationFrame> result = frames_;
    result.reserve(frames_.size() + queue_.size());
    for(const auto& frame : queue_) result.push_back(frame);
    return result;
}

tl::expected<void, std::string> ModelCalibrationRecorder::write_metadata() const {
    std::ofstream out(directory_ / "metadata.json");
    if(!out) return tl::make_unexpected(std::string("failed to open metadata.json"));
    out << std::setprecision(17) << "{\n";
    out << "  \"task_id\":\"" << json_escape(metadata_.task_id) << "\",\n";
    out << "  \"profile\":\"" << json_escape(metadata_.profile) << "\",\n";
    out << "  \"core_config_path\":\"" << json_escape(metadata_.core_config_path) << "\",\n";
    out << "  \"core_fingerprint\":\"" << json_escape(metadata_.core_fingerprint) << "\",\n";
    out << "  \"urdf_path\":\"" << json_escape(metadata_.urdf_path) << "\",\n";
    out << "  \"urdf_fingerprint\":\"" << json_escape(metadata_.urdf_fingerprint) << "\",\n";
    out << "  \"base_frame\":\"" << json_escape(metadata_.base_frame) << "\",\n";
    out << "  \"tool_frame\":\"" << json_escape(metadata_.tool_frame) << "\",\n";
    out << "  \"locked_joint_reference\":\"" << json_escape(metadata_.locked_joint_reference) << "\",\n";
    out << "  \"load_description\":\"" << json_escape(metadata_.load_description) << "\",\n";
    out << "  \"gravity\":[" << metadata_.gravity[0] << ',' << metadata_.gravity[1] << ',' << metadata_.gravity[2] << "],\n";
    out << "  \"joint_names\":[";
    for(std::size_t i=0;i<metadata_.joint_names.size();++i){if(i)out<<',';out<<'\"'<<json_escape(metadata_.joint_names[i])<<'\"';}
    out << "],\n  \"resource_paths\":[";
    for(std::size_t i=0;i<metadata_.resource_paths.size();++i){if(i)out<<',';out<<'\"'<<json_escape(metadata_.resource_paths[i])<<'\"';}
    out << "],\n  \"units\":{\"position\":\"" << json_escape(metadata_.position_unit) << "\",\"velocity\":\"" << json_escape(metadata_.velocity_unit) << "\",\"torque\":\"" << json_escape(metadata_.torque_unit) << "\"},\n";
    out << "  \"mapping\":{\"pos_ratio\":"; json_vector(out, metadata_.mapping_pos_ratio);
    out << ",\"tor_ratio\":"; json_vector(out, metadata_.mapping_tor_ratio);
    out << ",\"direction\":["; for(std::size_t i=0;i<metadata_.mapping_direction.size();++i){if(i)out<<',';out<<metadata_.mapping_direction[i];} out << ']';
    out << ",\"joint_zero_offset\":"; json_vector(out, metadata_.mapping_joint_zero_offset);
    out << ",\"actuator_zero_offset\":"; json_vector(out, metadata_.mapping_actuator_zero_offset); out << "},\n";
    out << "  \"original_gravity_scale\":"; json_vector(out, metadata_.original_gravity_scale); out << ",\n";
    out << "  \"calibration_options\":{\"mode\":\"" << json_escape(metadata_.identification_mode) << "\",\"locked_links\":[";
    for(std::size_t i=0;i<metadata_.locked_links.size();++i){if(i)out<<',';out<<'"'<<json_escape(metadata_.locked_links[i])<<'"';}
    out << "],\"identifiable_links\":[";
    for(std::size_t i=0;i<metadata_.identifiable_links.size();++i){if(i)out<<',';out<<'"'<<json_escape(metadata_.identifiable_links[i])<<'"';}
    out << "],\"pose_budget\":" << metadata_.pose_budget
        << ",\"validation_fraction\":" << metadata_.validation_fraction
        << ",\"max_com_offset_m\":" << metadata_.max_com_offset_m
        << ",\"regularization\":" << metadata_.regularization
        << ",\"svd_relative_threshold\":" << metadata_.svd_relative_threshold
        << ",\"minimum_information_score\":" << metadata_.minimum_information_score
        << ",\"max_task_duration_s\":" << metadata_.max_task_duration_s << "}\n}\n";
    if(!out) return tl::make_unexpected(std::string("failed to write metadata.json"));
    return {};
}

std::string ModelCalibrationRecorder::frame_line(const ModelCalibrationFrame& frame) const {
    std::ostringstream out;
    out << std::setprecision(17) << frame.monotonic_ns << ',' << frame.cycle << ',' << frame.phase << ',' << frame.direction << ',' << frame.speed_tier << ','
        << frame.pose_group << ',' << (frame.validation?1:0) << ',' << (frame.valid?1:0) << ',' << frame.feedback_age_ms << ',' << frame.acceleration_source << ',' << frame.impedance_mode;
    auto append = [&](const JointVector& values) { for(double value : values) out << ',' << value; };
    append(frame.position); append(frame.velocity); append(frame.torque); append(frame.acceleration); append(frame.reference_position); append(frame.reference_velocity);
    return out.str();
}

void ModelCalibrationRecorder::writer_loop() {
    std::ofstream out(directory_ / "frames.csv");
    if(!out) {
        std::lock_guard<std::mutex> lock(mutex_);
        status_.write_failed = true;
        status_.data_gap = true;
        status_.error = "failed to open frames.csv";
        running_.store(false);
        return;
    }
    out << "monotonic_ns,cycle,phase,direction,speed_tier,pose_group,validation,valid,feedback_age_ms,acceleration_source,impedance_mode";
    for(const char* prefix : {"q","dq","tau","acc","ref_q","ref_dq"}) for(const auto& name : metadata_.joint_names) out << ',' << prefix << ':' << name;
    out << '\n';
    while(true) {
        ModelCalibrationFrame frame;
        {
            std::unique_lock<std::mutex> lock(mutex_);
            cv_.wait(lock, [&]() { return stop_requested_.load() || !queue_.empty(); });
            if(queue_.empty() && stop_requested_.load()) break;
            frame = std::move(queue_.front());
            queue_.pop_front();
            frames_.push_back(frame);
        }
        out << frame_line(frame) << '\n';
        if(!out) {
            std::lock_guard<std::mutex> lock(mutex_);
            status_.write_failed = true;
            status_.data_gap = true;
            status_.error = "frames.csv write failed";
            break;
        }
        std::lock_guard<std::mutex> lock(mutex_);
        ++status_.written_frames;
    }
    out.flush();
    running_.store(false);
}

namespace {
tl::expected<void, std::string> write_trajectory_atomically(
    const std::filesystem::path& destination, const std::vector<std::string>& joints,
    const std::vector<JointVector>& trajectory, double sample_dt) {
    if(trajectory.size() < 20 || !std::isfinite(sample_dt) || sample_dt <= 0.0 || sample_dt > 0.5)
        return tl::make_unexpected(std::string("trajectory is too short or has invalid sample interval"));
    for(const auto& pose : trajectory)
        if(pose.size() != joints.size() || !finite_vector(pose))
            return tl::make_unexpected(std::string("trajectory contains invalid joint positions"));
    const auto temp = std::filesystem::path(destination.string() + ".tmp");
    {
        std::ofstream out(temp, std::ios::trunc);
        if(!out) return tl::make_unexpected(std::string("failed to open trajectory temporary file"));
        out << std::setprecision(17) << "sample_dt," << sample_dt << '\n';
        out << "index";
        for(const auto& name : joints) out << ',' << name;
        out << '\n';
        for(std::size_t i=0;i<trajectory.size();++i) {
            out << i;
            for(double value:trajectory[i]) out << ',' << value;
            out << '\n';
        }
        out.flush();
        if(!out) {
            std::error_code ignored;
            std::filesystem::remove(temp, ignored);
            return tl::make_unexpected(std::string("trajectory temporary write failed"));
        }
    }
    std::error_code error;
    std::filesystem::rename(temp, destination, error);
    if(error) {
        std::filesystem::remove(temp);
        return tl::make_unexpected(std::string("trajectory atomic rename failed: ") + error.message());
    }
    return {};
}
} // namespace

tl::expected<void, std::string> ModelCalibrationRecorder::write_trajectory(
    const std::vector<JointVector>& trajectory, double sample_dt) {
    std::filesystem::path destination;
    std::vector<std::string> joint_names;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(directory_.empty()) return tl::make_unexpected(std::string("recorder has no task directory"));
        destination = directory_ / "trajectory.csv";
        joint_names = metadata_.joint_names;
    }
    // Disk I/O must not hold the recorder lock used by the control cycle
    return write_trajectory_atomically(destination, joint_names, trajectory, sample_dt);
}

tl::expected<void, std::string> ModelCalibrationRecorder::write_trajectory_checkpoint(
    const std::vector<JointVector>& trajectory, double sample_dt) {
    std::filesystem::path destination;
    std::vector<std::string> joint_names;
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if(directory_.empty()) return tl::make_unexpected(std::string("recorder has no task directory"));
        destination = directory_ / "trajectory.checkpoint.csv";
        joint_names = metadata_.joint_names;
    }
    return write_trajectory_atomically(destination, joint_names, trajectory, sample_dt);
}

tl::expected<std::vector<ModelCalibrationFrame>, std::string> load_model_calibration_frames(
    const std::filesystem::path& directory,
    std::size_t joints_count) {
    std::ifstream in(directory / "frames.csv");
    if(!in) return tl::make_unexpected(std::string("failed to open frames.csv"));
    std::string line;
    if(!std::getline(in,line)) return tl::make_unexpected(std::string("frames.csv is empty"));
    std::vector<ModelCalibrationFrame> result;
    auto split=[](const std::string& text){std::vector<std::string> values;std::stringstream stream(text);std::string item;while(std::getline(stream,item,','))values.push_back(item);return values;};
    const std::size_t expected=11+6*joints_count;
    std::size_t line_number=1;
    while(std::getline(in,line)) {
        ++line_number;
        if(line.empty()) continue;
        const auto values=split(line);
        if(values.size()!=expected) return tl::make_unexpected("invalid frames.csv column count at line "+std::to_string(line_number));
        ModelCalibrationFrame frame;
        try {
            frame.monotonic_ns=static_cast<std::uint64_t>(std::stoull(values[0]));
            frame.cycle=static_cast<std::uint64_t>(std::stoull(values[1]));
            frame.phase=values[2];frame.direction=values[3];frame.speed_tier=values[4];frame.pose_group=static_cast<std::size_t>(std::stoull(values[5]));frame.validation=std::stoi(values[6])!=0;frame.valid=std::stoi(values[7])!=0;frame.feedback_age_ms=std::stod(values[8]);frame.acceleration_source=values[9];frame.impedance_mode=values[10];
            std::size_t offset=11;
            auto read_vector=[&](JointVector& target){target.resize(joints_count);for(std::size_t i=0;i<joints_count;++i)target[i]=std::stod(values[offset++]);};
            read_vector(frame.position);read_vector(frame.velocity);read_vector(frame.torque);read_vector(frame.acceleration);read_vector(frame.reference_position);read_vector(frame.reference_velocity);
        } catch(...) { return tl::make_unexpected("invalid numeric field in frames.csv at line "+std::to_string(line_number)); }
        if(!finite_vector(frame.position)||!finite_vector(frame.velocity)||!finite_vector(frame.torque)||!finite_vector(frame.acceleration)||!finite_vector(frame.reference_position)||!finite_vector(frame.reference_velocity)||!std::isfinite(frame.feedback_age_ms)) frame.valid=false;
        result.push_back(std::move(frame));
    }
    return result;
}

std::vector<GravityCalibrationPoseGroup> group_static_calibration_frames(const std::vector<ModelCalibrationFrame>& frames) {
    std::unordered_map<std::size_t, GravityCalibrationPoseGroup> grouped;
    for(const auto& frame : frames) {
        if(frame.pose_group == 0 || frame.phase.find("static") == std::string::npos) continue;
        auto& group = grouped[frame.pose_group];
        group.pose_group = frame.pose_group;
        group.validation = frame.validation;
        group.samples.push_back(frame);
    }
    // Earlier task records tagged the group before waiting for the robot to settle
    // Select the final 0.70 s of each group, matching the historic sample window
    // Fresh task records tag only the sample window and need no tail truncation
    for(auto& pair : grouped) {
        auto& samples = pair.second.samples;
        if(samples.empty()) continue;
        const auto last_ns = samples.back().monotonic_ns;
        constexpr std::uint64_t sample_window_ns = 700000000ULL;
        auto begin = std::lower_bound(samples.begin(), samples.end(),
            last_ns > sample_window_ns ? last_ns - sample_window_ns : 0ULL,
            [](const ModelCalibrationFrame& frame, std::uint64_t time) { return frame.monotonic_ns < time; });
        if(begin != samples.begin()) samples.erase(samples.begin(), begin);
        samples.erase(std::remove_if(samples.begin(), samples.end(), [](const ModelCalibrationFrame& frame) {
            if(!frame.valid || !finite_vector(frame.position) || !finite_vector(frame.velocity) || !finite_vector(frame.torque)) return true;
            // Quantized motor feedback can briefly exceed the settle velocity
            // Exclude genuine moving frames; never fit them as static gravity
            return std::any_of(frame.velocity.begin(), frame.velocity.end(), [](double speed) { return std::abs(speed) > 0.05; });
        }), samples.end());
    }
    std::vector<GravityCalibrationPoseGroup> result;
    result.reserve(grouped.size());
    for(auto& item : grouped) result.push_back(std::move(item.second));
    std::sort(result.begin(), result.end(), [](const auto& left, const auto& right) { return left.pose_group < right.pose_group; });
    return result;
}

tl::expected<GravityCalibrationResult, std::string> fit_gravity_calibration(
    Dynamics& dynamics,
    const std::vector<GravityCalibrationPoseGroup>& groups,
    const JointVector& original_gravity_scale,
    const GravityCalibrationOptions& options) {
    if(!dynamics.is_configured()) return tl::make_unexpected(std::string("Dynamics is not configured"));
    const auto poses = pose_statistics(groups);
    const std::size_t training_count = static_cast<std::size_t>(std::count_if(poses.begin(), poses.end(), [](const auto& pose){return !pose.validation;}));
    const std::size_t validation_count = poses.size() - training_count;
    if(training_count < options.minimum_training_groups) return tl::make_unexpected(std::string("insufficient training pose groups"));
    if(validation_count == 0) return tl::make_unexpected(std::string("validation pose groups are required"));

    const auto first = dynamics.get_gravity_regression(poses.front().q);
    if(!first) return tl::make_unexpected(std::string("gravity regression failed"));
    const std::size_t joints = first->original_gravity.size();
    const std::size_t total_params = static_cast<std::size_t>(first->first_moment_regressor.cols());
    if(joints == 0 || total_params == 0 || total_params != first->links.size()*3)
        return tl::make_unexpected(std::string("gravity regression has invalid parameter dimensions"));
    if(options.identification_mode != "global" && options.identification_mode != "local")
        return tl::make_unexpected(std::string("unknown gravity identification mode"));
    if(options.identification_mode == "global" && !options.locked_links.empty())
        return tl::make_unexpected(std::string("global identification cannot specify locked links"));
    std::set<std::string> locked(options.locked_links.begin(), options.locked_links.end());
    if(locked.size() != options.locked_links.size())
        return tl::make_unexpected(std::string("duplicate locked link"));
    std::vector<Eigen::Index> active_columns;
    std::vector<std::string> fitted_links;
    for(std::size_t i=0;i<first->links.size();++i) {
        const auto& name=first->links[i].link_name;
        if(locked.erase(name) == 0) {
            fitted_links.push_back(name);
            for(std::size_t axis=0;axis<3;++axis)active_columns.push_back(static_cast<Eigen::Index>(3*i+axis));
        }
    }
    if(!locked.empty())return tl::make_unexpected(std::string("locked link does not belong to current identifiable model: ")+*locked.begin());
    if(active_columns.empty())return tl::make_unexpected(std::string("all identifiable links are locked"));
    const std::size_t params = active_columns.size();

    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(training_count*joints), static_cast<Eigen::Index>(params+joints));
    Eigen::VectorXd b = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(training_count*joints));
    std::size_t row = 0;
    double reconstruction_sum = 0.0;
    std::size_t reconstruction_n = 0;
    for(const auto& pose : poses) {
        if(pose.validation) continue;
        const auto regression = dynamics.get_gravity_regression(pose.q);
        if(!regression || static_cast<std::size_t>(regression->first_moment_regressor.cols()) != total_params) return tl::make_unexpected(std::string("gravity regression shape changed"));
        reconstruction_sum += regression->reconstruction_rms * regression->reconstruction_rms;
        ++reconstruction_n;
        for(std::size_t joint=0;joint<joints;++joint,++row) {
            const double sigma = std::max(0.01, pose.tau_std[joint]);
            const double weight = 1.0 / sigma;
            for(std::size_t col=0;col<params;++col) A(static_cast<Eigen::Index>(row),static_cast<Eigen::Index>(col)) = weight * regression->first_moment_regressor(static_cast<Eigen::Index>(joint),active_columns[col]);
            A(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(params+joint)) = -weight;
            b(static_cast<Eigen::Index>(row)) = weight * (pose.tau[joint] - regression->original_gravity[joint]);
        }
    }

    // Fit from pose-to-pose torque variations.  Constant joint offsets are
    // nuisance parameters; joint-centering removes them without consuming
    // first-moment observability.  This also prevents a torque zero-offset
    // from masquerading as a link COM displacement.
    Eigen::MatrixXd centered = A.leftCols(static_cast<Eigen::Index>(params));
    Eigen::VectorXd target = b;
    // Retain the uncentered regressor energy as a numerical reference.  A
    // column whose pose-to-pose variation is only roundoff is *not* an
    // observable direction, no matter how large it becomes after scaling.
    Eigen::VectorXd uncentered_energy = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(params));
    for(std::size_t joint=0;joint<joints;++joint) {
        for(std::size_t pose=0;pose<training_count;++pose) {
            const Eigen::Index r=static_cast<Eigen::Index>(pose*joints+joint);
            const double weight = -A(r,static_cast<Eigen::Index>(params+joint));
            if(!(weight>0.0) || !std::isfinite(weight)) return tl::make_unexpected(std::string("invalid gravity regression weight"));
            centered.row(r) /= weight;
            target(r) /= weight;
            uncentered_energy.array() += centered.row(r).transpose().array().square();
        }

        // Subtract a reference pose before computing the mean.  Directly
        // subtracting the mean of nearly identical nonzero rows introduces
        // cancellation noise and used to give repeated poses a fake rank.
        const Eigen::RowVectorXd reference = centered.row(static_cast<Eigen::Index>(joint));
        const double target_reference = target(static_cast<Eigen::Index>(joint));
        Eigen::RowVectorXd mean_delta = Eigen::RowVectorXd::Zero(static_cast<Eigen::Index>(params));
        double target_mean_delta = 0.0;
        for(std::size_t pose=0;pose<training_count;++pose) {
            const Eigen::Index r=static_cast<Eigen::Index>(pose*joints+joint);
            centered.row(r) -= reference;
            target(r) -= target_reference;
            mean_delta += centered.row(r);
            target_mean_delta += target(r);
        }
        mean_delta /= static_cast<double>(training_count);
        target_mean_delta /= static_cast<double>(training_count);
        for(std::size_t pose=0;pose<training_count;++pose) {
            const Eigen::Index r=static_cast<Eigen::Index>(pose*joints+joint);
            centered.row(r) -= mean_delta;
            target(r) -= target_mean_delta;
        }
    }

    // Normalize by observed regressor energy, NEVER by a COM offset bound.
    // Tiny/zero columns are unobservable: they retain the source URDF prior.
    Eigen::VectorXd column_scale(static_cast<Eigen::Index>(params));
    Eigen::MatrixXd normalized=centered;
    for(std::size_t col=0;col<params;++col) {
        const double energy=centered.col(static_cast<Eigen::Index>(col)).squaredNorm()/static_cast<double>(centered.rows());
        const double rms=std::sqrt(std::max(0.0,energy));
        const double original_rms=std::sqrt(std::max(0.0,uncentered_energy(static_cast<Eigen::Index>(col))/static_cast<double>(centered.rows())));
        // The relative term rejects floating-point cancellation, while the
        // absolute floor avoids amplifying sub-picometre-equivalent columns.
        // This is an observability check, NOT a COM-displacement constraint.
        const double excitation_floor=std::max(1.0e-12,1.0e-9*original_rms);
        if(!(rms>excitation_floor)) {
            normalized.col(static_cast<Eigen::Index>(col)).setZero();
            column_scale(static_cast<Eigen::Index>(col))=1.0;
            continue;
        }
        const double inv=1.0/rms;
        column_scale(static_cast<Eigen::Index>(col))=inv;
        normalized.col(static_cast<Eigen::Index>(col))*=inv;
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> base_svd(normalized,Eigen::ComputeThinU|Eigen::ComputeThinV);
    GravityCalibrationResult result;
    result.regression_reconstruction_rms=reconstruction_n?std::sqrt(reconstruction_sum/static_cast<double>(reconstruction_n)):0.0;
    result.singular_values.resize(static_cast<std::size_t>(base_svd.singularValues().size()));
    for(Eigen::Index i=0;i<base_svd.singularValues().size();++i)result.singular_values[static_cast<std::size_t>(i)]=base_svd.singularValues()[i];
    const double max_sv=base_svd.singularValues().size()?base_svd.singularValues()[0]:0.0;
    result.numerical_rank=0;
    for(Eigen::Index i=0;i<base_svd.singularValues().size();++i)
        if(max_sv>0.0 && base_svd.singularValues()[i]>max_sv*options.svd_relative_threshold)
            ++result.numerical_rank;
    result.identification_mode=options.identification_mode;
    result.locked_links=options.locked_links;
    result.fitted_links=fitted_links;
    result.parameter_observable.assign(total_params,0);
    if(result.numerical_rank) {
        const auto V=base_svd.matrixV();
        for(std::size_t col=0;col<params;++col) {
            double projection=0.0;
            for(std::size_t k=0;k<result.numerical_rank;++k)
                projection+=std::pow(V(static_cast<Eigen::Index>(col),static_cast<Eigen::Index>(k)),2);
            if(projection>=options.minimum_observable_energy)result.parameter_observable[static_cast<std::size_t>(active_columns[col])]=1;
        }
    }

    // Select ridge strength using ONLY the training pose groups
    // Whole-pose leave-one-out avoids leaking any held-out evaluation poses
    // The unchanged URDF is a valid model-selection baseline when excitation
    // does not support a transferable correction
    std::vector<Eigen::MatrixXd> pose_regressors(training_count,
        Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(joints), static_cast<Eigen::Index>(params)));
    std::vector<Eigen::VectorXd> pose_targets(training_count,
        Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joints)));
    for(std::size_t pose=0;pose<training_count;++pose) {
        for(std::size_t joint=0;joint<joints;++joint) {
            const Eigen::Index index=static_cast<Eigen::Index>(pose*joints+joint);
            const double w=-A(index,static_cast<Eigen::Index>(params+joint));
            pose_regressors[pose].row(static_cast<Eigen::Index>(joint)) =
                A.block(index,0,1,static_cast<Eigen::Index>(params))/w;
            pose_targets[pose](static_cast<Eigen::Index>(joint))=b(index)/w;
        }
    }
    const std::vector<double> candidate_lambdas{std::max(0.0,options.regularization),
        std::max(0.1,options.regularization),std::max(1.0,options.regularization),
        std::max(10.0,options.regularization),std::max(100.0,options.regularization),
        std::max(1000.0,options.regularization)};
    double best_cv_error=std::numeric_limits<double>::infinity();
    double chosen_lambda=candidate_lambdas.front();
    bool prefer_prior=true;
    for(std::size_t setting=0;setting<=candidate_lambdas.size();++setting) {
        const bool prior=setting==0;
        const double lambda=prior?0.0:candidate_lambdas[setting-1];
        double cv_error=0.0;
        for(std::size_t held=0;held<training_count;++held) {
            const Eigen::Index rows=static_cast<Eigen::Index>((training_count-1)*joints);
            Eigen::MatrixXd sample_A=Eigen::MatrixXd::Zero(rows,static_cast<Eigen::Index>(params));
            Eigen::VectorXd sample_b=Eigen::VectorXd::Zero(rows);
            Eigen::MatrixXd mean_Y=Eigen::MatrixXd::Zero(static_cast<Eigen::Index>(joints),static_cast<Eigen::Index>(params));
            Eigen::VectorXd mean_b=Eigen::VectorXd::Zero(static_cast<Eigen::Index>(joints));
            for(std::size_t p=0;p<training_count;++p) {
                if(p==held)continue;
                mean_Y+=pose_regressors[p];
                mean_b+=pose_targets[p];
            }
            mean_Y/=static_cast<double>(training_count-1);
            mean_b/=static_cast<double>(training_count-1);
            Eigen::Index offset=0;
            for(std::size_t p=0;p<training_count;++p) {
                if(p==held)continue;
                sample_A.middleRows(offset,static_cast<Eigen::Index>(joints))=pose_regressors[p]-mean_Y;
                sample_b.segment(offset,static_cast<Eigen::Index>(joints))=pose_targets[p]-mean_b;
                offset+=static_cast<Eigen::Index>(joints);
            }
            Eigen::VectorXd fold_delta=Eigen::VectorXd::Zero(static_cast<Eigen::Index>(params));
            if(!prior) {
                Eigen::VectorXd scaling=Eigen::VectorXd::Ones(static_cast<Eigen::Index>(params));
                for(std::size_t c=0;c<params;++c) {
                    const Eigen::Index col=static_cast<Eigen::Index>(c);
                    const double rms=sample_A.col(col).norm()/std::sqrt(static_cast<double>(rows));
                    const double reference=pose_regressors[0].col(col).norm()/std::sqrt(static_cast<double>(joints));
                    if(rms>std::max(1.0e-12,1.0e-9*reference)) {
                        sample_A.col(col)/=rms;
                        scaling[col]=1.0/rms;
                    } else sample_A.col(col).setZero();
                }
                Eigen::JacobiSVD<Eigen::MatrixXd> fold_svd(sample_A,Eigen::ComputeThinU|Eigen::ComputeThinV);
                Eigen::VectorXd coeff=fold_svd.matrixU().transpose()*sample_b;
                const double largest=fold_svd.singularValues().size()?fold_svd.singularValues()[0]:0.0;
                for(Eigen::Index k=0;k<coeff.size();++k) {
                    const double sigma=fold_svd.singularValues()[k];
                    if(sigma<=largest*options.svd_relative_threshold || sigma<=1.0e-12) coeff[k]=0.0;
                    else coeff[k]*=sigma/(sigma*sigma+lambda);
                }
                fold_delta=scaling.cwiseProduct(fold_svd.matrixV()*coeff);
            }
            const Eigen::VectorXd error=(pose_regressors[held]-mean_Y)*fold_delta-
                (pose_targets[held]-mean_b);
            cv_error+=error.squaredNorm();
        }
        if(cv_error+1.0e-10<best_cv_error) {
            best_cv_error=cv_error;
            chosen_lambda=lambda;
            prefer_prior=prior;
        }
    }
    result.selected_regularization=chosen_lambda;
    result.prior_preferred=prefer_prior;

    // Robust iteratively reweighted SVD ridge in the observable subspace.
    // Huber reweighting reduces the influence of transient torque spikes.
    // Ridge is a soft numerical prior, not a hard COM or output constraint.
    Eigen::VectorXd normalized_delta=Eigen::VectorXd::Zero(static_cast<Eigen::Index>(params));
    Eigen::VectorXd weights=Eigen::VectorXd::Ones(target.size());
    for(std::size_t iter=0;iter<5 && result.numerical_rank>0 && !prefer_prior;++iter) {
        Eigen::MatrixXd weighted=normalized;
        Eigen::VectorXd weighted_b=target;
        for(Eigen::Index r=0;r<weighted.rows();++r) {
            const double w=std::sqrt(weights[r]);
            weighted.row(r)*=w;
            weighted_b[r]*=w;
        }
        Eigen::JacobiSVD<Eigen::MatrixXd> fit_svd(weighted,Eigen::ComputeThinU|Eigen::ComputeThinV);
        Eigen::VectorXd coeff=fit_svd.matrixU().transpose()*weighted_b;
        const double largest=fit_svd.singularValues().size()?fit_svd.singularValues()[0]:0.0;
        for(Eigen::Index k=0;k<coeff.size();++k) {
            const double sigma=fit_svd.singularValues()[k];
            if(sigma<=largest*options.svd_relative_threshold || sigma<=1.0e-12)coeff[k]=0.0;
            else coeff[k]*=sigma/(sigma*sigma+chosen_lambda);
        }
        normalized_delta=fit_svd.matrixV()*coeff;
        const Eigen::VectorXd residual=normalized*normalized_delta-target;
        // Per-joint robust scale: do not downweight entire joint groups merely
        // because their expected gravitational torque is larger.
        for(std::size_t joint=0;joint<joints;++joint) {
            std::vector<double> abs_residual;
            for(std::size_t pose=0;pose<training_count;++pose)
                abs_residual.push_back(std::abs(residual[static_cast<Eigen::Index>(pose*joints+joint)]));
            const double mad=percentile_abs(abs_residual,0.5);
            const double huber=std::max(0.05,1.345*1.4826*mad);
            for(std::size_t pose=0;pose<training_count;++pose) {
                const Eigen::Index r=static_cast<Eigen::Index>(pose*joints+joint);
                weights[r]=std::min(1.0,huber/std::max(huber,std::abs(residual[r])));
            }
        }
    }
    const Eigen::VectorXd delta_h=column_scale.cwiseProduct(normalized_delta);
    if(!delta_h.allFinite()) return tl::make_unexpected(std::string("gravity fit produced non-finite first moments"));
    result.first_moments.reserve(first->links.size());
    result.constraints_ok = true; // Legacy result field: no COM displacement bound is enforced.
    result.com_offsets_m.reserve(first->links.size());
    std::size_t active_link=0;
    for(std::size_t link=0;link<first->links.size();++link) {
        const auto& info=first->links[link];
        if(!(info.mass>0.0) || !std::isfinite(info.mass))
            return tl::make_unexpected(std::string("invalid link mass for ")+info.link_name);
        Eigen::Vector3d shift=Eigen::Vector3d::Zero();
        if(std::find(fitted_links.begin(),fitted_links.end(),info.link_name)!=fitted_links.end()) {
            shift=delta_h.segment<3>(static_cast<Eigen::Index>(3*active_link))/info.mass;
            ++active_link;
        }
        if(!shift.allFinite())return tl::make_unexpected(std::string("non-finite candidate COM for ")+info.link_name);
        result.com_offsets_m.push_back(GravityFirstMoment{info.link_name,shift});
        result.first_moments.push_back(GravityFirstMoment{info.link_name,info.mass*(info.center_of_mass+shift)});
    }
    const JointVector original_bias=estimate_model_bias(dynamics,poses,nullptr,nullptr);
    const JointVector scaled_bias=estimate_model_bias(dynamics,poses,nullptr,&original_gravity_scale);
    const JointVector candidate_bias=estimate_model_bias(dynamics,poses,&result.first_moments,nullptr);
    result.training_original=evaluate_model(dynamics,poses,false,nullptr,nullptr,original_bias);
    result.training_scaled=evaluate_model(dynamics,poses,false,nullptr,&original_gravity_scale,scaled_bias);
    result.training_candidate=evaluate_model(dynamics,poses,false,&result.first_moments,nullptr,candidate_bias);
    result.validation_original=evaluate_model(dynamics,poses,true,nullptr,nullptr,original_bias);
    result.validation_scaled=evaluate_model(dynamics,poses,true,nullptr,&original_gravity_scale,scaled_bias);
    result.validation_candidate=evaluate_model(dynamics,poses,true,&result.first_moments,nullptr,candidate_bias);
    result.torque_bias=candidate_bias;
    result.noise_rms.assign(joints,0.0);
    std::size_t noise_count=0;
    for(const auto& pose:poses){if(pose.validation)continue;++noise_count;for(std::size_t j=0;j<joints;++j)result.noise_rms[j]+=pose.tau_std[j]*pose.tau_std[j];}
    if(noise_count)for(double& value:result.noise_rms)value=std::sqrt(value/static_cast<double>(noise_count));

    // Holdout quality is advisory but must be honest: low absolute RMS is NOT
    // permission to worsen an already-good model by a large relative factor
    // Per-axis sensor resolution gives a small tolerance for numerical noise
    bool acceptable=result.numerical_rank>0;
    double original_squared=0.0;
    double candidate_squared=0.0;
    for(std::size_t j=0;j<joints;++j) {
        const double original=result.validation_original.rms[j];
        const double candidate=result.validation_candidate.rms[j];
        const double permitted=std::max(0.005,0.05*original);
        if(!std::isfinite(candidate) || candidate>original+permitted) acceptable=false;
        original_squared+=original*original;
        candidate_squared+=candidate*candidate;
    }
    if(candidate_squared>original_squared+1.0e-12) acceptable=false;
    result.static_pass=acceptable && !result.prior_preferred;
    if(result.numerical_rank==0) {result.status="failed";result.failure_reason="gravity_parameters_unobservable";}
    else if(result.prior_preferred) {result.status="needs_review";result.failure_reason="training_pose_cross_validation_prefers_urdf_prior";}
    else if(!result.static_pass) {result.status="needs_review";result.failure_reason="holdout_validation_not_improved";}
    else {result.status="passed";}
    return result;
}

std::string gravity_calibration_result_json(const GravityCalibrationResult& result) {
    std::ostringstream out;
    out << std::setprecision(17) << "{\"status\":\"" << json_escape(result.status) << "\",\"failure_reason\":\"" << json_escape(result.failure_reason) << "\"";
    out << ",\"static_pass\":" << (result.static_pass?"true":"false") << ",\"constraints_ok\":" << (result.constraints_ok?"true":"false") << ",\"numerical_rank\":" << result.numerical_rank;
    out << ",\"identification_mode\":\"" << json_escape(result.identification_mode) << "\",\"locked_links\":[";
    for(std::size_t i=0;i<result.locked_links.size();++i){if(i)out<<',';out<<'"'<<json_escape(result.locked_links[i])<<'"';}
    out << "],\"fitted_links\":[";
    for(std::size_t i=0;i<result.fitted_links.size();++i){if(i)out<<',';out<<'"'<<json_escape(result.fitted_links[i])<<'"';}
    out << "],\"selected_regularization\":" << result.selected_regularization << ",\"prior_preferred\":" << (result.prior_preferred?"true":"false");
    out << ",\"regression_reconstruction_rms\":" << result.regression_reconstruction_rms << ",\"singular_values\":[";
    for(std::size_t i=0;i<result.singular_values.size();++i){if(i)out<<',';out<<result.singular_values[i];} out<<']';
    out << ",\"parameter_observable\":[";for(std::size_t i=0;i<result.parameter_observable.size();++i){if(i)out<<',';out<<static_cast<int>(result.parameter_observable[i]);}out<<']';
    out << ",\"torque_bias\":";json_vector(out,result.torque_bias);
    out << ",\"first_moments\":[";
    for(std::size_t i=0;i<result.first_moments.size();++i){if(i)out<<',';const auto& item=result.first_moments[i];out<<"{\"link_name\":\""<<json_escape(item.link_name)<<"\",\"value\":["<<item.value.x()<<','<<item.value.y()<<','<<item.value.z()<<"]}";}out<<']';
    out<<",\"com_offsets_m\":[";
    for(std::size_t i=0;i<result.com_offsets_m.size();++i){if(i)out<<',';const auto& item=result.com_offsets_m[i];out<<"{\"link_name\":\""<<json_escape(item.link_name)<<"\",\"value\":["<<item.value.x()<<','<<item.value.y()<<','<<item.value.z()<<"]}";}out<<']';
    out<<",\"solver\":\"robust_centered_svd_ridge_unbounded\"";
    auto metrics=[&](const char* name,const GravityValidationMetrics& m){out<<",\""<<name<<"\":{\"rms\":";json_vector(out,m.rms);out<<",\"p99\":";json_vector(out,m.p99);out<<",\"max\":";json_vector(out,m.maximum);out<<",\"bias\":";json_vector(out,m.bias);out<<'}';};
    metrics("training_original",result.training_original);metrics("training_scaled",result.training_scaled);metrics("training_candidate",result.training_candidate);metrics("validation_original",result.validation_original);metrics("validation_scaled",result.validation_scaled);metrics("validation_candidate",result.validation_candidate);
    out<<",\"noise_rms\":";json_vector(out,result.noise_rms);out<<'}';return out.str();
}

std::string model_calibration_file_fingerprint(const std::string& path) {
    std::ifstream in(path, std::ios::binary);
    if(!in) return {};
    std::uint64_t hash=1469598103934665603ULL;
    char buffer[4096];
    while(in){in.read(buffer,sizeof(buffer));const auto count=in.gcount();for(std::streamsize i=0;i<count;++i){hash^=static_cast<unsigned char>(buffer[i]);hash*=1099511628211ULL;}}
    std::ostringstream out;out<<std::hex<<std::setw(16)<<std::setfill('0')<<hash;return out.str();
}

} // namespace serial_arm
