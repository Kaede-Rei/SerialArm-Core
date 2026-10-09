#include "serial_arm/dynamics/dynamics.hpp"

#include <pinocchio/algorithm/aba.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>
#include <regex>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace serial_arm {

// ! ========================= 宏 定 义 ========================= ! //



// ! ========================= 接 口 变 量 ========================= ! //



// ! ========================= 私 有 量 / 工 具 函 数 实 现 ========================= ! //

struct Dynamics::Impl {
    DynamicsCfg cfg;        ///< 动力学配置
    DynamicsInfo info;      ///< 动力学模型信息
    DynamicsState state;    ///< 最近一次成功更新的动力学缓存

    pinocchio::Model model;                 ///< Pinocchio 模型对象
    std::unique_ptr<pinocchio::Data> data;  ///< Pinocchio 数据对象

    std::vector<int> q_indices;     ///< 受控关节位置索引
    std::vector<int> v_indices;     ///< 受控关节速度索引

    pinocchio::FrameIndex base_frame_id{ 0 };   ///< base_frame 索引
    pinocchio::FrameIndex tool_frame_id{ 0 };   ///< tool_frame 索引

    Eigen::VectorXd q_model;        ///< 完整模型位置向量
    Eigen::VectorXd dq_model;       ///< 完整模型速度向量
    Eigen::VectorXd ddq_model;      ///< 完整模型参考加速度向量
    Eigen::VectorXd tau_model;      ///< 完整模型反馈力矩向量

    Eigen::MatrixXd frame_jacobian_model;           ///< 完整模型 Frame Jacobian 临时缓存
    std::vector<Eigen::Isometry3d> frame_poses;     ///< 所有 Frame 位姿缓存
    std::vector<Eigen::MatrixXd> frame_jacobians;   ///< 所有 Frame Jacobian 缓存
    std::vector<GravityLinkParameterInfo> gravity_links; ///< 可辨识的受控关节 Link 参数
    std::vector<GravityLinkParameterInfo> gravity_reference_links; ///< 全部真实 Link 用于回归重建核对
    std::unordered_map<std::string, std::size_t> gravity_link_index; ///< Link 名称到参数索引
    std::vector<GravityFirstMoment> gravity_override; ///< 当前候选一阶矩
    bool gravity_override_active{ false }; ///< 是否启用候选重力覆盖

    bool is_configured{ false };    ///< 是否已经完成配置
    bool is_updated{ false };       ///< 是否已经完成至少一次更新
};

namespace {


/**
 * @brief 读取文本文件
 */
std::string read_text_file(const std::string& path) {
    std::ifstream stream(path);
    if(!stream) return {};
    std::ostringstream out;
    out << stream.rdbuf();
    return out.str();
}

/**
 * @brief 从 XML 标签文本读取属性
 */
std::optional<std::string> xml_attribute(const std::string& text, const std::string& name) {
    const std::regex pattern(name + R"(\s*=\s*[\"']([^\"']*)[\"'])", std::regex::icase);
    std::smatch match;
    if(!std::regex_search(text, match, pattern)) return std::nullopt;
    return match[1].str();
}

/**
 * @brief 解析空格分隔的三维向量
 */
bool parse_vec3_text(const std::string& text, Eigen::Vector3d& value) {
    std::istringstream stream(text);
    double x = 0.0, y = 0.0, z = 0.0;
    if(!(stream >> x >> y >> z)) return false;
    std::string extra;
    if(stream >> extra) return false;
    if(!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
    value = Eigen::Vector3d(x, y, z);
    return true;
}

/**
 * @brief 解析 URDF 原始 Link 惯性参数
 */
std::vector<GravityLinkParameterInfo> parse_urdf_gravity_links(const std::string& urdf_path) {
    std::vector<GravityLinkParameterInfo> links;
    const std::string xml = read_text_file(urdf_path);
    if(xml.empty()) return links;
    const std::regex link_pattern(R"(<link\b([^>]*)>([\s\S]*?)</link\s*>)", std::regex::icase);
    const std::regex inertial_pattern(R"(<inertial\b[^>]*>([\s\S]*?)</inertial\s*>)", std::regex::icase);
    const std::regex origin_pattern(R"(<origin\b([^>]*)/?>)", std::regex::icase);
    const std::regex mass_pattern(R"(<mass\b([^>]*)/?>)", std::regex::icase);
    const std::regex inertia_pattern(R"(<inertia\b([^>]*)/?>)", std::regex::icase);
    for(std::sregex_iterator it(xml.begin(), xml.end(), link_pattern), end; it != end; ++it) {
        const auto name = xml_attribute((*it)[1].str(), "name");
        if(!name || name->empty()) continue;
        std::smatch inertial_match;
        const std::string body = (*it)[2].str();
        if(!std::regex_search(body, inertial_match, inertial_pattern)) continue;
        const std::string inertial = inertial_match[1].str();
        GravityLinkParameterInfo info;
        info.link_name = *name;
        Eigen::Vector3d xyz = Eigen::Vector3d::Zero();
        Eigen::Vector3d rpy = Eigen::Vector3d::Zero();
        std::smatch match;
        if(std::regex_search(inertial, match, origin_pattern)) {
            const auto xyz_text = xml_attribute(match[1].str(), "xyz");
            const auto rpy_text = xml_attribute(match[1].str(), "rpy");
            if(xyz_text && !parse_vec3_text(*xyz_text, xyz)) continue;
            if(rpy_text && !parse_vec3_text(*rpy_text, rpy)) continue;
        }
        if(!std::regex_search(inertial, match, mass_pattern)) continue;
        const auto mass_text = xml_attribute(match[1].str(), "value");
        if(!mass_text) continue;
        try { info.mass = std::stod(*mass_text); }
        catch(...) { continue; }
        if(!std::isfinite(info.mass) || info.mass <= 0.0) continue;
        if(!std::regex_search(inertial, match, inertia_pattern)) continue;
        const std::string inertia_attrs = match[1].str();
        auto get = [&](const char* key, double& out) {
            const auto value = xml_attribute(inertia_attrs, key);
            if(!value) return false;
            try { out = std::stod(*value); }
            catch(...) { return false; }
            return std::isfinite(out);
        };
        double ixx=0.0, ixy=0.0, ixz=0.0, iyy=0.0, iyz=0.0, izz=0.0;
        if(!get("ixx",ixx)||!get("ixy",ixy)||!get("ixz",ixz)||!get("iyy",iyy)||!get("iyz",iyz)||!get("izz",izz)) continue;
        Eigen::Matrix3d inertia_matrix;
        inertia_matrix << ixx, ixy, ixz, ixy, iyy, iyz, ixz, iyz, izz;
        const Eigen::AngleAxisd roll(rpy.x(), Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd pitch(rpy.y(), Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd yaw(rpy.z(), Eigen::Vector3d::UnitZ());
        const Eigen::Matrix3d rotation = (yaw * pitch * roll).toRotationMatrix();
        info.center_of_mass = xyz;
        info.inertia = rotation * inertia_matrix * rotation.transpose();
        const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eig(info.inertia);
        info.inertia_valid = eig.info() == Eigen::Success && eig.eigenvalues().minCoeff() >= -1.0e-10;
        links.push_back(std::move(info));
    }
    return links;
}

/**
 * @brief 将完整模型向量按受控关节索引提取为 JointVector
 */
JointVector extract_controlled(const Eigen::VectorXd& source, const std::vector<int>& indices) {
    JointVector result(indices.size(), 0.0);
    for(std::size_t i = 0; i < indices.size(); ++i) result[i] = source[indices[i]];
    return result;
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
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> validate_joint_vector(const JointVector& values, std::size_t expected_size) {
    if(values.size() != expected_size) {
        return tl::make_unexpected(DynamicsErr::INVALID_INPUT_SIZE);
    }

    if(!finite_vector(values)) {
        return tl::make_unexpected(DynamicsErr::NON_FINITE_INPUT);
    }

    return {};
}

/**
 * @brief 验证重力补偿缩放系数
 * @param gravity_scale 重力补偿缩放系数
 * @param expected_size 期望大小
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> validate_gravity_scale(const JointVector& gravity_scale, std::size_t expected_size) {
    if(gravity_scale.size() != expected_size) {
        return tl::make_unexpected(DynamicsErr::INVALID_INPUT_SIZE);
    }

    const bool finite = std::all_of(gravity_scale.begin(), gravity_scale.end(), [](double value) {
        return std::isfinite(value);
        });
    if(!finite) return tl::make_unexpected(DynamicsErr::NON_FINITE_INPUT);

    const bool in_range = std::all_of(gravity_scale.begin(), gravity_scale.end(), [](double value) {
        return value >= 0.0 && value <= 2.0;
        });
    if(!in_range) return tl::make_unexpected(DynamicsErr::GRAVITY_SCALE_OUT_OF_RANGE);

    return {};
}

/**
 * @brief 将关节向量写入完整模型向量
 * @param source 关节向量
 * @param indices 完整模型索引
 * @param target 完整模型向量
 */
void assign_joint_vector(const JointVector& source, const std::vector<int>& indices, Eigen::VectorXd& target) {
    target.setZero();

    for(std::size_t i = 0; i < source.size(); ++i) {
        target[indices[i]] = source[i];
    }
}

/**
 * @brief 从完整模型向量提取受控关节向量
 * @param source 完整模型向量
 * @param indices 完整模型索引
 * @param target 受控关节向量
 */
void extract_joint_vector(const Eigen::VectorXd& source, const std::vector<int>& indices, JointVector& target) {
    if(target.size() != indices.size()) {
        target.resize(indices.size());
    }

    for(std::size_t i = 0; i < indices.size(); ++i) {
        target[i] = source[indices[i]];
    }
}

/**
 * @brief 从完整质量矩阵提取受控关节子矩阵
 * @param source 完整质量矩阵
 * @param indices 受控关节速度索引
 * @param target 受控关节质量矩阵
 */
void extract_joint_matrix(const Eigen::MatrixXd& source, const std::vector<int>& indices, Eigen::MatrixXd& target) {
    const Eigen::Index size = static_cast<Eigen::Index>(indices.size());
    if(target.rows() != size || target.cols() != size) {
        target.resize(size, size);
    }

    for(std::size_t row = 0; row < indices.size(); ++row) {
        for(std::size_t col = 0; col < indices.size(); ++col) {
            target(static_cast<Eigen::Index>(row), static_cast<Eigen::Index>(col)) = source(indices[row], indices[col]);
        }
    }
}

/**
 * @brief 从完整 Frame Jacobian 提取受控关节列
 * @param source 完整 Frame Jacobian
 * @param indices 受控关节速度索引
 * @param target 受控关节 Frame Jacobian
 */
void extract_frame_jacobian(const Eigen::MatrixXd& source, const std::vector<int>& indices, Eigen::MatrixXd& target) {
    const Eigen::Index cols = static_cast<Eigen::Index>(indices.size());
    if(target.rows() != 6 || target.cols() != cols) {
        target.resize(6, cols);
    }

    for(std::size_t col = 0; col < indices.size(); ++col) {
        target.col(static_cast<Eigen::Index>(col)) = source.col(indices[col]);
    }
}

/**
 * @brief 将 Pinocchio SE3 转换为 Eigen Isometry3d
 * @param source Pinocchio 位姿
 * @return Eigen 位姿
 */
Eigen::Isometry3d to_isometry(const pinocchio::SE3& source) {
    Eigen::Isometry3d output = Eigen::Isometry3d::Identity();
    output.linear() = source.rotation();
    output.translation() = source.translation();
    return output;
}

} // namespace

// ! ========================= 接 口 类 方 法 / 函 数 实 现 ========================= ! //

/**
 * @brief 构造一个尚未配置的动力学对象
 */
Dynamics::Dynamics() : impl_(std::make_unique<Impl>()) {

}

/**
 * @brief 析构动力学对象并释放内部模型资源
 */
Dynamics::~Dynamics() = default;

/**
 * @brief 移动构造动力学对象
 * @param other 被移动的动力学对象
 */
Dynamics::Dynamics(Dynamics&& other) noexcept = default;

/**
 * @brief 移动赋值动力学对象
 * @param other 被移动的动力学对象
 * @return 当前对象引用
 */
Dynamics& Dynamics::operator=(Dynamics&& other) noexcept = default;

/**
 * @brief 根据配置加载并初始化动力学模型
 * @param cfg 动力学配置
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> Dynamics::configure(const DynamicsCfg& cfg) {
    if(impl_->is_configured) {
        return tl::make_unexpected(DynamicsErr::ALREADY_CONFIGURED);
    }

    if(cfg.urdf_path.empty() || cfg.joint_names.empty() || cfg.base_frame.empty() || cfg.tool_frame.empty() ||
        !std::isfinite(cfg.gravity[0]) || !std::isfinite(cfg.gravity[1]) || !std::isfinite(cfg.gravity[2])) {
        return tl::make_unexpected(DynamicsErr::INVALID_CFG);
    }

    JointVector gravity_scale = cfg.gravity_scale;
    if(gravity_scale.empty()) {
        gravity_scale.assign(cfg.joint_names.size(), 0.0);
    }

    const auto gravity_scale_valid = validate_gravity_scale(gravity_scale, cfg.joint_names.size());
    if(!gravity_scale_valid) {
        return tl::make_unexpected(DynamicsErr::INVALID_CFG);
    }

    if(!std::filesystem::exists(cfg.urdf_path)) {
        return tl::make_unexpected(DynamicsErr::URDF_LOAD_FAILED);
    }

    std::unordered_set<std::string> unique_names;
    for(const auto& name : cfg.joint_names) {
        if(name.empty() || !unique_names.insert(name).second) {
            return tl::make_unexpected(DynamicsErr::INVALID_CFG);
        }
    }

    pinocchio::Model full_model;
    try {
        pinocchio::urdf::buildModel(cfg.urdf_path, full_model);
    }
    catch(...) {
        return tl::make_unexpected(DynamicsErr::URDF_LOAD_FAILED);
    }

    for(const auto& name : cfg.joint_names) {
        const pinocchio::JointIndex joint_id = full_model.getJointId(name);
        if(joint_id == 0 || static_cast<int>(joint_id) >= full_model.njoints) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_FOUND);
        }
        if(full_model.nqs[joint_id] != 1 || full_model.nvs[joint_id] != 1) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_1DOF);
        }
    }

    const double original_total_mass = pinocchio::computeTotalMass(full_model);

    std::unordered_set<std::string> controlled(cfg.joint_names.begin(), cfg.joint_names.end());
    std::vector<pinocchio::JointIndex> joints_to_lock;
    for(pinocchio::JointIndex joint_id = 1; static_cast<int>(joint_id) < full_model.njoints; ++joint_id) {
        if(controlled.find(full_model.names[joint_id]) == controlled.end()) {
            joints_to_lock.push_back(joint_id);
        }
    }

    pinocchio::Model reduced_model;
    try {
        reduced_model = pinocchio::buildReducedModel(full_model, joints_to_lock, pinocchio::neutral(full_model));
    }
    catch(...) {
        return tl::make_unexpected(DynamicsErr::URDF_LOAD_FAILED);
    }

    const int expected_size = static_cast<int>(cfg.joint_names.size());
    if(reduced_model.nq != expected_size || reduced_model.nv != expected_size) {
        return tl::make_unexpected(DynamicsErr::MODEL_SIZE_MISMATCH);
    }

    reduced_model.gravity.linear() = Eigen::Vector3d(cfg.gravity[0], cfg.gravity[1], cfg.gravity[2]);

    std::vector<int> q_indices(cfg.joint_names.size(), -1);
    std::vector<int> v_indices(cfg.joint_names.size(), -1);
    for(std::size_t i = 0; i < cfg.joint_names.size(); ++i) {
        const pinocchio::JointIndex joint_id = reduced_model.getJointId(cfg.joint_names[i]);
        if(joint_id == 0 || static_cast<int>(joint_id) >= reduced_model.njoints) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_FOUND);
        }
        if(reduced_model.nqs[joint_id] != 1 || reduced_model.nvs[joint_id] != 1) {
            return tl::make_unexpected(DynamicsErr::JOINT_NOT_1DOF);
        }

        q_indices[i] = reduced_model.idx_qs[joint_id];
        v_indices[i] = reduced_model.idx_vs[joint_id];
    }

    if(!reduced_model.existFrame(cfg.base_frame) || !reduced_model.existFrame(cfg.tool_frame)) {
        return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);
    }

    const pinocchio::FrameIndex base_frame_id = reduced_model.getFrameId(cfg.base_frame);
    const pinocchio::FrameIndex tool_frame_id = reduced_model.getFrameId(cfg.tool_frame);
    if(static_cast<int>(base_frame_id) >= reduced_model.nframes || static_cast<int>(tool_frame_id) >= reduced_model.nframes) {
        return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);
    }

    impl_->cfg = cfg;
    impl_->cfg.gravity_scale = std::move(gravity_scale);
    impl_->model = std::move(reduced_model);
    impl_->data = std::make_unique<pinocchio::Data>(impl_->model);
    impl_->q_indices = std::move(q_indices);
    impl_->v_indices = std::move(v_indices);
    impl_->base_frame_id = base_frame_id;
    impl_->tool_frame_id = tool_frame_id;

    impl_->q_model = Eigen::VectorXd::Zero(impl_->model.nq);
    impl_->dq_model = Eigen::VectorXd::Zero(impl_->model.nv);
    impl_->ddq_model = Eigen::VectorXd::Zero(impl_->model.nv);
    impl_->tau_model = Eigen::VectorXd::Zero(impl_->model.nv);
    impl_->frame_jacobian_model = Eigen::MatrixXd::Zero(6, impl_->model.nv);

    const std::size_t joints_count = cfg.joint_names.size();
    impl_->state.pos.assign(joints_count, 0.0);
    impl_->state.vel.assign(joints_count, 0.0);
    impl_->state.acc.assign(joints_count, 0.0);
    impl_->state.tor.assign(joints_count, 0.0);
    impl_->state.ref_acc.assign(joints_count, 0.0);
    impl_->state.gravity.assign(joints_count, 0.0);
    impl_->state.candidate_gravity.assign(joints_count, 0.0);
    impl_->state.effective_gravity.assign(joints_count, 0.0);
    impl_->state.gravity_compensation.assign(joints_count, 0.0);
    impl_->state.gravity_override_active = false;
    impl_->state.nonlinear.assign(joints_count, 0.0);
    impl_->state.coriolis.assign(joints_count, 0.0);
    impl_->state.inverse_dynamics.assign(joints_count, 0.0);
    impl_->state.forward_dynamics.assign(joints_count, 0.0);
    impl_->state.mass_matrix = Eigen::MatrixXd::Zero(expected_size, expected_size);
    impl_->state.center_of_mass = Eigen::Vector3d::Zero();
    impl_->state.tool_pose = Eigen::Isometry3d::Identity();
    impl_->state.tool_jacobian = Eigen::MatrixXd::Zero(6, expected_size);

    impl_->frame_poses.assign(impl_->model.nframes, Eigen::Isometry3d::Identity());
    impl_->frame_jacobians.reserve(impl_->model.nframes);
    for(pinocchio::FrameIndex frame_id = 0; static_cast<int>(frame_id) < impl_->model.nframes; ++frame_id) {
        impl_->frame_jacobians.emplace_back(Eigen::MatrixXd::Zero(6, expected_size));
    }

    impl_->gravity_links.clear();
    impl_->gravity_reference_links.clear();
    impl_->gravity_link_index.clear();
    // Fit only inertial links attached directly to a sampled controlled joint
    // Fixed coordinates and unsampled auxiliary joints keep the recorded URDF prior
    const std::string urdf_xml = read_text_file(cfg.urdf_path);
    std::unordered_set<std::string> identifiable_links;
    const std::regex joint_pattern(R"(<joint\b([^>]*)>([\s\S]*?)</joint\s*>)", std::regex::icase);
    const std::regex child_pattern(R"(<child\b([^>]*)/?>)", std::regex::icase);
    for(std::sregex_iterator it(urdf_xml.begin(), urdf_xml.end(), joint_pattern), end; it != end; ++it) {
        const auto name = xml_attribute((*it)[1].str(), "name");
        if(!name || controlled.find(*name) == controlled.end()) continue;
        std::smatch child;
        const std::string body = (*it)[2].str();
        if(std::regex_search(body, child, child_pattern)) {
            const auto link = xml_attribute(child[1].str(), "link");
            if(link) identifiable_links.insert(*link);
        }
    }
    for(auto info : parse_urdf_gravity_links(cfg.urdf_path)) {
        if(!impl_->model.existFrame(info.link_name)) continue;
        const pinocchio::FrameIndex frame_id = impl_->model.getFrameId(info.link_name);
        if(static_cast<int>(frame_id) >= impl_->model.nframes) continue;
        if(identifiable_links.find(info.link_name) != identifiable_links.end()) {
            impl_->gravity_link_index.emplace(info.link_name, impl_->gravity_links.size());
            impl_->gravity_links.push_back(info);
        }
        impl_->gravity_reference_links.push_back(std::move(info));
    }
    impl_->gravity_override.clear();
    impl_->gravity_override_active = false;

    impl_->info.joints_count = joints_count;
    impl_->info.nq = impl_->model.nq;
    impl_->info.nv = impl_->model.nv;
    impl_->info.total_mass = original_total_mass;
    impl_->info.reduced_total_mass = pinocchio::computeTotalMass(impl_->model);
    impl_->info.effective_moving_mass = 0.0;
    impl_->info.base_frame = cfg.base_frame;
    impl_->info.tool_frame = cfg.tool_frame;
    impl_->info.joint_names = cfg.joint_names;
    impl_->info.q_indices = impl_->q_indices;
    impl_->info.v_indices = impl_->v_indices;
    impl_->info.frame_names.clear();
    impl_->info.frame_names.reserve(impl_->model.frames.size());
    for(const auto& frame : impl_->model.frames) impl_->info.frame_names.push_back(frame.name);
    impl_->info.effective_inertias.clear();
    impl_->info.effective_inertias.reserve(cfg.joint_names.size());
    for(const auto& name : cfg.joint_names) {
        const pinocchio::JointIndex joint_id = impl_->model.getJointId(name);
        const pinocchio::Inertia& inertia = impl_->model.inertias[joint_id];
        impl_->info.effective_moving_mass += inertia.mass();
        DynamicsInertiaInfo item;
        item.joint_name = name;
        item.mass = inertia.mass();
        item.center_of_mass = inertia.lever();
        item.inertia = inertia.inertia();
        impl_->info.effective_inertias.push_back(std::move(item));
    }

    impl_->is_configured = true;
    impl_->is_updated = false;
    return {};
}

/**
 * @brief 集中更新当前周期的全部运动学与动力学缓存
 * @param state 当前关节位置、速度和反馈力矩
 * @param acc 当前关节加速度估计
 * @param ref_acc 当前关节参考加速度
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> Dynamics::update(const JointState& state, const JointVector& acc, const JointVector& ref_acc) {
    const auto state_result = update_state(state, acc);
    if(!state_result) {
        return tl::make_unexpected(state_result.error());
    }

    const auto reference_result = update_reference(ref_acc);
    if(!reference_result) {
        return tl::make_unexpected(reference_result.error());
    }

    return {};
}

/**
 * @brief 更新当前状态对应的运动学与动力学缓存
 * @param state 当前关节位置、速度和反馈力矩
 * @param acc 当前关节加速度估计
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> Dynamics::update_state(const JointState& state, const JointVector& acc) {
    if(!is_configured()) {
        return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    }

    const auto pos_valid = validate_joint_vector(state.pos, impl_->info.joints_count);
    const auto vel_valid = validate_joint_vector(state.vel, impl_->info.joints_count);
    const auto tor_valid = validate_joint_vector(state.tor, impl_->info.joints_count);
    const auto acc_valid = validate_joint_vector(acc, impl_->info.joints_count);
    if(!pos_valid) return tl::make_unexpected(pos_valid.error());
    if(!vel_valid) return tl::make_unexpected(vel_valid.error());
    if(!tor_valid) return tl::make_unexpected(tor_valid.error());
    if(!acc_valid) return tl::make_unexpected(acc_valid.error());

    assign_joint_vector(state.pos, impl_->q_indices, impl_->q_model);
    assign_joint_vector(state.vel, impl_->v_indices, impl_->dq_model);
    assign_joint_vector(state.tor, impl_->v_indices, impl_->tau_model);

    impl_->is_updated = false;

    try {
        pinocchio::computeAllTerms(impl_->model, *impl_->data, impl_->q_model, impl_->dq_model);
        pinocchio::updateFramePlacements(impl_->model, *impl_->data);

        const pinocchio::SE3 base_pose_inverse = impl_->data->oMf[impl_->base_frame_id].inverse();
        const Eigen::Vector3d center_world = pinocchio::centerOfMass(impl_->model, *impl_->data, impl_->q_model, false);
        impl_->state.center_of_mass = base_pose_inverse.rotation() * center_world + base_pose_inverse.translation();
        for(pinocchio::FrameIndex frame_id = 0; static_cast<int>(frame_id) < impl_->model.nframes; ++frame_id) {
            impl_->frame_poses[frame_id] = to_isometry(base_pose_inverse * impl_->data->oMf[frame_id]);
            impl_->frame_jacobian_model.setZero();
            pinocchio::getFrameJacobian(impl_->model, *impl_->data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, impl_->frame_jacobian_model);
            extract_frame_jacobian(impl_->frame_jacobian_model, impl_->v_indices, impl_->frame_jacobians[frame_id]);
        }

        extract_joint_vector(impl_->data->g, impl_->v_indices, impl_->state.gravity);
        extract_joint_vector(impl_->data->nle, impl_->v_indices, impl_->state.nonlinear);
        impl_->state.candidate_gravity = impl_->state.gravity;
        if(impl_->gravity_override_active) {
            for(const auto& candidate : impl_->gravity_override) {
                const auto index_it = impl_->gravity_link_index.find(candidate.link_name);
                if(index_it == impl_->gravity_link_index.end()) continue;
                const auto& original = impl_->gravity_links[index_it->second];
                const pinocchio::FrameIndex frame_id = impl_->model.getFrameId(candidate.link_name);
                if(static_cast<int>(frame_id) >= impl_->model.nframes) continue;
                const Eigen::Vector3d delta_h = candidate.value - original.mass * original.center_of_mass;
                const Eigen::Matrix3d rotation = impl_->data->oMf[frame_id].rotation();
                const Eigen::Vector3d gravity_world = impl_->model.gravity.linear();
                const Eigen::Vector3d moment = gravity_world.cross(rotation * delta_h);
                const Eigen::MatrixXd& jacobian = impl_->frame_jacobians[frame_id];
                // Pinocchio Motion/Jacobian rows are [linear; angular]
                for(std::size_t joint = 0; joint < impl_->info.joints_count; ++joint) {
                    impl_->state.candidate_gravity[joint] += jacobian.block<3,1>(3, static_cast<Eigen::Index>(joint)).dot(moment);
                }
            }
        }
        impl_->state.gravity_override_active = impl_->gravity_override_active;
        for(std::size_t i = 0; i < impl_->info.joints_count; ++i) {
            impl_->state.effective_gravity[i] = impl_->gravity_override_active ?
                impl_->state.candidate_gravity[i] : impl_->cfg.gravity_scale[i] * impl_->state.gravity[i];
            impl_->state.gravity_compensation[i] = impl_->state.effective_gravity[i];
            impl_->state.coriolis[i] = impl_->state.nonlinear[i] - impl_->state.gravity[i];
        }

        impl_->data->M.triangularView<Eigen::StrictlyLower>() = impl_->data->M.transpose().triangularView<Eigen::StrictlyLower>();
        extract_joint_matrix(impl_->data->M, impl_->v_indices, impl_->state.mass_matrix);

        const Eigen::VectorXd& forward_dynamics = pinocchio::aba(impl_->model, *impl_->data, impl_->q_model, impl_->dq_model, impl_->tau_model);
        extract_joint_vector(forward_dynamics, impl_->v_indices, impl_->state.forward_dynamics);
    }
    catch(...) {
        return tl::make_unexpected(DynamicsErr::COMPUTE_FAILED);
    }

    impl_->state.pos = state.pos;
    impl_->state.vel = state.vel;
    impl_->state.acc = acc;
    impl_->state.tor = state.tor;
    std::fill(impl_->state.ref_acc.begin(), impl_->state.ref_acc.end(), 0.0);
    std::fill(impl_->state.inverse_dynamics.begin(), impl_->state.inverse_dynamics.end(), 0.0);
    impl_->state.tool_pose = impl_->frame_poses[impl_->tool_frame_id];
    impl_->state.tool_jacobian = impl_->frame_jacobians[impl_->tool_frame_id];
    impl_->is_updated = true;
    return {};
}

/**
 * @brief 使用当前状态缓存更新参考逆动力学
 * @param ref_acc 当前关节参考加速度
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> Dynamics::update_reference(const JointVector& ref_acc) {
    if(!is_configured()) {
        return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    }
    if(!is_updated()) {
        return tl::make_unexpected(DynamicsErr::NOT_UPDATED);
    }

    const auto ref_acc_valid = validate_joint_vector(ref_acc, impl_->info.joints_count);
    if(!ref_acc_valid) return tl::make_unexpected(ref_acc_valid.error());

    assign_joint_vector(ref_acc, impl_->v_indices, impl_->ddq_model);

    try {
        const Eigen::VectorXd& inverse_dynamics = pinocchio::rnea(impl_->model, *impl_->data, impl_->q_model, impl_->dq_model, impl_->ddq_model);
        extract_joint_vector(inverse_dynamics, impl_->v_indices, impl_->state.inverse_dynamics);
        // gravity_scale 是一次性机械臂动力学标定结果；FULL_INVERSE_DYNAMICS 也必须
        // 使用同一套校准后的重力项，否则 HOLD 与 TRACKING 的 residual 基线会跳变
        for(std::size_t i = 0; i < impl_->info.joints_count; ++i) {
            impl_->state.inverse_dynamics[i] +=
                impl_->state.gravity_compensation[i] - impl_->state.gravity[i];
        }
    }
    catch(...) {
        return tl::make_unexpected(DynamicsErr::COMPUTE_FAILED);
    }

    impl_->state.ref_acc = ref_acc;
    return {};
}

/**
 * @brief 更新重力补偿缩放系数
 * @param gravity_scale 重力补偿缩放系数
 * @return 成功时返回空值；失败时返回 DynamicsErr
 */
tl::expected<void, DynamicsErr> Dynamics::set_gravity_scale(const JointVector& gravity_scale) {
    if(!is_configured()) {
        return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    }

    const auto valid = validate_gravity_scale(gravity_scale, impl_->info.joints_count);
    if(!valid) {
        return tl::make_unexpected(valid.error());
    }

    impl_->cfg.gravity_scale = gravity_scale;
    if(impl_->is_updated && !impl_->gravity_override_active) {
        for(std::size_t i = 0; i < impl_->info.joints_count; ++i) {
            impl_->state.effective_gravity[i] = impl_->cfg.gravity_scale[i] * impl_->state.gravity[i];
            impl_->state.gravity_compensation[i] = impl_->state.effective_gravity[i];
        }
    }
    return {};
}

/**
 * @brief 清理当前动力学模型并恢复未配置状态
 */
void Dynamics::cleanup() {
    impl_ = std::make_unique<Impl>();
}

/**
 * @brief 查询动力学模型是否已经完成配置
 */
bool Dynamics::is_configured() const noexcept {
    return impl_ && impl_->is_configured;
}

/**
 * @brief 查询动力学缓存是否已经完成至少一次成功更新
 */
bool Dynamics::is_updated() const noexcept {
    return impl_ && impl_->is_updated;
}

/**
 * @brief 获取当前动力学模型的基本信息
 */
const DynamicsInfo& Dynamics::get_info() const noexcept {
    return impl_->info;
}

/**
 * @brief 获取最近一次 update() 的完整缓存
 */
const DynamicsState& Dynamics::get_state() const noexcept {
    return impl_->state;
}

/**
 * @brief 获取当前重力补偿缩放系数
 */
const JointVector& Dynamics::get_gravity_scale() const noexcept {
    return impl_->cfg.gravity_scale;
}

/**
 * @brief 获取指定坐标系相对于 base_frame 的缓存位姿
 */
tl::expected<Eigen::Isometry3d, DynamicsErr> Dynamics::get_frame_pose(const std::string& frame_name) const {
    if(!is_configured()) return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    if(!impl_->is_updated) return tl::make_unexpected(DynamicsErr::NOT_UPDATED);
    if(frame_name.empty() || !impl_->model.existFrame(frame_name)) return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);

    const pinocchio::FrameIndex frame_id = impl_->model.getFrameId(frame_name);
    if(frame_id >= impl_->frame_poses.size()) return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);
    return impl_->frame_poses[frame_id];
}

/**
 * @brief 获取指定坐标系的缓存几何 Jacobian
 */
tl::expected<Eigen::MatrixXd, DynamicsErr> Dynamics::get_frame_jacobian(const std::string& frame_name) const {
    if(!is_configured()) return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    if(!impl_->is_updated) return tl::make_unexpected(DynamicsErr::NOT_UPDATED);
    if(frame_name.empty() || !impl_->model.existFrame(frame_name)) return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);

    const pinocchio::FrameIndex frame_id = impl_->model.getFrameId(frame_name);
    if(frame_id >= impl_->frame_jacobians.size()) return tl::make_unexpected(DynamicsErr::FRAME_NOT_FOUND);
    return impl_->frame_jacobians[frame_id];
}

/**
 * @brief 获取最近一次 update() 的未缩放重力广义力
 */
const JointVector& Dynamics::get_gravity() const noexcept {
    return impl_->state.gravity;
}

/**
 * @brief 获取最近一次 update() 的缩放后重力补偿
 */
tl::expected<GravityRegressionResult, DynamicsErr> Dynamics::get_gravity_regression(const JointVector& positions) const {
    if(!is_configured()) return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    const auto valid = validate_joint_vector(positions, impl_->info.joints_count);
    if(!valid) return tl::make_unexpected(valid.error());
    try {
        Eigen::VectorXd q_model = Eigen::VectorXd::Zero(impl_->model.nq);
        assign_joint_vector(positions, impl_->q_indices, q_model);
        Eigen::VectorXd dq_model = Eigen::VectorXd::Zero(impl_->model.nv);
        pinocchio::Data data(impl_->model);
        pinocchio::computeAllTerms(impl_->model, data, q_model, dq_model);
        pinocchio::updateFramePlacements(impl_->model, data);
        GravityRegressionResult result;
        result.original_gravity = extract_controlled(data.g, impl_->v_indices);
        result.links = impl_->gravity_links;
        result.first_moment_regressor = Eigen::MatrixXd::Zero(
            static_cast<Eigen::Index>(impl_->info.joints_count),
            static_cast<Eigen::Index>(3 * result.links.size()));
        Eigen::VectorXd reconstructed = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(impl_->info.joints_count));
        const Eigen::Vector3d gravity_world = impl_->model.gravity.linear();
        Eigen::MatrixXd jacobian_model = Eigen::MatrixXd::Zero(6, impl_->model.nv);
        for(const auto& link : impl_->gravity_reference_links) {
            const auto fit_index = impl_->gravity_link_index.find(link.link_name);
            const pinocchio::FrameIndex frame_id = impl_->model.getFrameId(link.link_name);
            if(static_cast<int>(frame_id) >= impl_->model.nframes) continue;
            jacobian_model.setZero();
            pinocchio::getFrameJacobian(impl_->model, data, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, jacobian_model);
            Eigen::MatrixXd jacobian;
            extract_frame_jacobian(jacobian_model, impl_->v_indices, jacobian);
            const Eigen::Matrix3d rotation = data.oMf[frame_id].rotation();
            const Eigen::Vector3d h = link.mass * link.center_of_mass;
            const Eigen::Vector3d force = link.mass * gravity_world;
            for(std::size_t joint = 0; joint < impl_->info.joints_count; ++joint) {
                // Pinocchio Motion/Jacobian rows are [linear; angular]
                const Eigen::Vector3d linear = jacobian.block<3,1>(0, static_cast<Eigen::Index>(joint));
                const Eigen::Vector3d angular = jacobian.block<3,1>(3, static_cast<Eigen::Index>(joint));
                reconstructed(static_cast<Eigen::Index>(joint)) += -linear.dot(force) + angular.dot(gravity_world.cross(rotation * h));
                if(fit_index != impl_->gravity_link_index.end()) {
                    for(int axis = 0; axis < 3; ++axis) {
                        const Eigen::Vector3d basis = rotation.col(axis);
                        result.first_moment_regressor(
                            static_cast<Eigen::Index>(joint),
                            static_cast<Eigen::Index>(3 * fit_index->second + static_cast<std::size_t>(axis))) =
                            angular.dot(gravity_world.cross(basis));
                    }
                }
            }
        }
        double error_sq = 0.0;
        for(std::size_t joint = 0; joint < impl_->info.joints_count; ++joint) {
            const double error = reconstructed(static_cast<Eigen::Index>(joint)) - result.original_gravity[joint];
            error_sq += error * error;
        }
        result.reconstruction_rms = impl_->info.joints_count ?
            std::sqrt(error_sq / static_cast<double>(impl_->info.joints_count)) : 0.0;
        return result;
    }
    catch(...) { return tl::make_unexpected(DynamicsErr::COMPUTE_FAILED); }
}

tl::expected<JointVector, DynamicsErr> Dynamics::compute_gravity_with_first_moments(
    const JointVector& positions,
    const std::vector<GravityFirstMoment>& first_moments) const {
    const auto regression = get_gravity_regression(positions);
    if(!regression) return tl::make_unexpected(regression.error());
    Eigen::VectorXd delta = Eigen::VectorXd::Zero(static_cast<Eigen::Index>(3 * regression->links.size()));
    std::unordered_map<std::string, Eigen::Vector3d> candidates;
    for(const auto& item : first_moments) {
        if(!item.value.allFinite()) return tl::make_unexpected(DynamicsErr::NON_FINITE_INPUT);
        candidates[item.link_name] = item.value;
    }
    for(std::size_t i = 0; i < regression->links.size(); ++i) {
        const auto& link = regression->links[i];
        const auto it = candidates.find(link.link_name);
        if(it == candidates.end()) continue;
        delta.segment<3>(static_cast<Eigen::Index>(3 * i)) = it->second - link.mass * link.center_of_mass;
    }
    Eigen::VectorXd gravity = Eigen::Map<const Eigen::VectorXd>(regression->original_gravity.data(), static_cast<Eigen::Index>(regression->original_gravity.size()));
    gravity += regression->first_moment_regressor * delta;
    JointVector result(static_cast<std::size_t>(gravity.size()), 0.0);
    for(Eigen::Index i = 0; i < gravity.size(); ++i) result[static_cast<std::size_t>(i)] = gravity[i];
    return result;
}

tl::expected<void, DynamicsErr> Dynamics::set_gravity_first_moment_override(
    const std::vector<GravityFirstMoment>& first_moments) {
    if(!is_configured()) return tl::make_unexpected(DynamicsErr::NOT_CONFIGURED);
    std::unordered_set<std::string> names;
    for(const auto& item : first_moments) {
        if(!item.value.allFinite() || impl_->gravity_link_index.find(item.link_name) == impl_->gravity_link_index.end() ||
            !names.insert(item.link_name).second) return tl::make_unexpected(DynamicsErr::INVALID_CFG);
    }
    impl_->gravity_override = first_moments;
    impl_->gravity_override_active = !first_moments.empty();
    if(impl_->is_updated) {
        const auto candidate = compute_gravity_with_first_moments(impl_->state.pos, impl_->gravity_override);
        if(!candidate) return tl::make_unexpected(candidate.error());
        impl_->state.candidate_gravity = candidate.value();
        impl_->state.gravity_override_active = impl_->gravity_override_active;
        for(std::size_t i = 0; i < impl_->info.joints_count; ++i) {
            impl_->state.effective_gravity[i] = impl_->gravity_override_active ? impl_->state.candidate_gravity[i] : impl_->cfg.gravity_scale[i] * impl_->state.gravity[i];
            impl_->state.gravity_compensation[i] = impl_->state.effective_gravity[i];
        }
    }
    return {};
}

void Dynamics::clear_gravity_first_moment_override() {
    if(!impl_) return;
    impl_->gravity_override.clear();
    impl_->gravity_override_active = false;
    if(impl_->is_updated) {
        impl_->state.candidate_gravity = impl_->state.gravity;
        impl_->state.gravity_override_active = false;
        for(std::size_t i = 0; i < impl_->info.joints_count; ++i) {
            impl_->state.effective_gravity[i] = impl_->cfg.gravity_scale[i] * impl_->state.gravity[i];
            impl_->state.gravity_compensation[i] = impl_->state.effective_gravity[i];
        }
    }
}

bool Dynamics::has_gravity_first_moment_override() const noexcept {
    return impl_ && impl_->gravity_override_active;
}

const std::vector<GravityFirstMoment>& Dynamics::get_gravity_first_moment_override() const noexcept {
    return impl_->gravity_override;
}

const JointVector& Dynamics::get_effective_gravity() const noexcept {
    return impl_->state.effective_gravity;
}

const JointVector& Dynamics::get_candidate_gravity() const noexcept {
    return impl_->state.candidate_gravity;
}

const JointVector& Dynamics::get_gravity_compensation() const noexcept {
    return impl_->state.gravity_compensation;
}

/**
 * @brief 获取最近一次 update() 的完整非线性广义力
 */
const JointVector& Dynamics::get_nonlinear() const noexcept {
    return impl_->state.nonlinear;
}

/**
 * @brief 获取最近一次 update() 的科氏力和离心力广义力
 */
const JointVector& Dynamics::get_coriolis() const noexcept {
    return impl_->state.coriolis;
}

/**
 * @brief 获取最近一次 update() 的关节空间质量矩阵
 */
const Eigen::MatrixXd& Dynamics::get_mass_matrix() const noexcept {
    return impl_->state.mass_matrix;
}

/**
 * @brief 获取最近一次 update() 的逆动力学结果
 */
const JointVector& Dynamics::get_inverse_dynamics() const noexcept {
    return impl_->state.inverse_dynamics;
}

/**
 * @brief 获取最近一次 update() 的正向动力学结果
 */
const JointVector& Dynamics::get_forward_dynamics() const noexcept {
    return impl_->state.forward_dynamics;
}

/**
 * @brief 获取最近一次 update() 的末端位姿
 */
const Eigen::Isometry3d& Dynamics::get_tool_pose() const noexcept {
    return impl_->state.tool_pose;
}

/**
 * @brief 获取最近一次 update() 的末端 Jacobian
 */
const Eigen::MatrixXd& Dynamics::get_tool_jacobian() const noexcept {
    return impl_->state.tool_jacobian;
}

// ! ========================= 私 有 类 方 法 实 现 ========================= ! //



} // namespace serial_arm
