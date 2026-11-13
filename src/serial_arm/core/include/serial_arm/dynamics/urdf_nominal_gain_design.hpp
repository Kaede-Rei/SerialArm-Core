#pragma once
#include <Eigen/Cholesky>
#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace serial_arm::servo_tuning {
struct UrdfNominalModalDiagnostics {
    // Undamped K*phi=omega^2*M*phi modes, ordered by frequency. These are
    // ideal local rigid-body model properties, never a measured bandwidth.
    std::vector<double> nominal_mode_frequency_hz,nominal_mode_damping_ratio;
    // With non-diagonal Phi^T*D*Phi, the ratios are modal projections only;
    // they do not describe independent damped modes of the coupled system.
    bool damping_projection_is_exact{false};
};
inline UrdfNominalModalDiagnostics ideal_urdf_modal_diagnostics(
    const std::vector<std::vector<double>>& mass_matrix,
    const std::vector<double>& kp,const std::vector<double>& kd) {
    const auto n=mass_matrix.size();
    if(!n || kp.size()!=n || kd.size()!=n)
        throw std::invalid_argument("URDF modal diagnostics require the complete mapped mass matrix and gain group");
    Eigen::MatrixXd M(n,n);
    Eigen::VectorXd P(n),D(n);
    for(std::size_t i=0;i<n;++i) {
        if(mass_matrix[i].size()!=n || !std::isfinite(kp[i]) || kp[i]<0 || !std::isfinite(kd[i]) || kd[i]<0)
            throw std::invalid_argument("URDF modal diagnostics require finite nonnegative mapped gains");
        P(i)=kp[i];D(i)=kd[i];
        for(std::size_t j=0;j<n;++j)M(i,j)=mass_matrix[i][j];
    }
    if(!M.allFinite() || !M.isApprox(M.transpose(),1e-10) || Eigen::LLT<Eigen::MatrixXd>(M).info()!=Eigen::Success)
        throw std::invalid_argument("URDF modal diagnostics require a finite symmetric positive definite mass matrix");
    const Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> modes(P.asDiagonal().toDenseMatrix(),M);
    if(modes.info()!=Eigen::Success)
        throw std::invalid_argument("URDF nominal modal eigensolve failed");
    const Eigen::MatrixXd modal_D=modes.eigenvectors().transpose()*D.asDiagonal()*modes.eigenvectors();
    Eigen::MatrixXd coupling=modal_D;coupling.diagonal().setZero();
    UrdfNominalModalDiagnostics out;
    out.damping_projection_is_exact=coupling.norm()<=1e-9*std::max(1e-14,modal_D.norm());
    for(std::size_t i=0;i<n;++i) {
        const double squared=std::max(0.0,modes.eigenvalues()(i));
        const double omega=std::sqrt(squared);
        const auto phi=modes.eigenvectors().col(i);
        out.nominal_mode_frequency_hz.push_back(omega/(2*3.14159265358979323846));
        out.nominal_mode_damping_ratio.push_back(omega>0?
            modal_D(i,i)/(2*omega*phi.dot(M*phi)):std::numeric_limits<double>::quiet_NaN());
    }
    return out;
}
namespace urdf_nominal_detail {
inline Eigen::MatrixXd checked_mass_matrix(const std::vector<std::vector<double>>& matrix) {
    const auto n=matrix.size();
    if(!n)throw std::invalid_argument("URDF nominal design requires a complete mapped mass matrix");
    Eigen::MatrixXd M(n,n);
    for(std::size_t i=0;i<n;++i) {
        if(matrix[i].size()!=n)throw std::invalid_argument("URDF mass matrix joint mapping mismatch");
        for(std::size_t j=0;j<n;++j)M(i,j)=matrix[i][j];
    }
    if(!M.allFinite() || !M.isApprox(M.transpose(),1e-10) || Eigen::LLT<Eigen::MatrixXd>(M).info()!=Eigen::Success)
        throw std::invalid_argument("URDF mass matrix must be finite, symmetric and positive definite; repair geometry/inertial data or joint mapping");
    return M;
}
}

struct UrdfNominalDampingDesign {
    std::vector<double> stiffness_basis,unconstrained_kd;
    std::vector<std::vector<double>> mass_matrix_basis,full_damping_matrix;
    std::string damping_design_method{"double_diagonalization_diagonal_projection"};
    std::string damping_interface_projection{"retain_diagonal_for_independent_mit_derivative_channels"};
    double damping_ratio{.7};
    UrdfNominalModalDiagnostics modal_diagnostics;
    bool measured{false};
};

// Final complete K is an explicit input. In particular, callers constrain or
// encode P first and then design D from exactly that K and this same M basis.
// No response frequency, inertia majorant or earlier P is silently rebuilt.
inline UrdfNominalDampingDesign nominal_urdf_damping_design(
    const std::vector<std::vector<double>>& mass_matrix,const std::vector<double>& final_complete_kp,
    double damping_ratio=.7) {
    const auto M=urdf_nominal_detail::checked_mass_matrix(mass_matrix);const auto n=mass_matrix.size();
    if(final_complete_kp.size()!=n || !std::isfinite(damping_ratio) || damping_ratio<=0 ||
       std::any_of(final_complete_kp.begin(),final_complete_kp.end(),[](double p){return !std::isfinite(p) || p<=0;}))
        throw std::invalid_argument("URDF nominal damping requires the final complete positive stiffness group and damping target");
    const Eigen::Map<const Eigen::VectorXd> P(final_complete_kp.data(),n);
    const Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> modes(P.asDiagonal().toDenseMatrix(),M);
    if(modes.info()!=Eigen::Success || !modes.eigenvalues().allFinite() || modes.eigenvalues().minCoeff()<=0)
        throw std::invalid_argument("URDF nominal damping eigensolve failed; repair inertial data");
    const Eigen::VectorXd modal_damping=2*damping_ratio*modes.eigenvalues().array().sqrt().matrix();
    const Eigen::MatrixXd full=M*modes.eigenvectors()*modal_damping.asDiagonal()*modes.eigenvectors().transpose()*M;
    if(!full.allFinite() || full.diagonal().minCoeff()<=0)
        throw std::invalid_argument("URDF nominal damping overflow; repair inertial data");
    UrdfNominalDampingDesign out;out.mass_matrix_basis=mass_matrix;out.stiffness_basis=final_complete_kp;
    out.damping_ratio=damping_ratio;out.full_damping_matrix.assign(n,std::vector<double>(n));
    for(std::size_t i=0;i<n;++i) {
        for(std::size_t j=0;j<n;++j)out.full_damping_matrix[i][j]=.5*full(i,j)+.5*full(j,i);
        out.unconstrained_kd.push_back(full(i,i));
    }
    out.modal_diagnostics=ideal_urdf_modal_diagnostics(mass_matrix,final_complete_kp,out.unconstrained_kd);
    return out;
}

struct UrdfPathInertiaBasis {
    std::vector<std::vector<double>> mass_matrix_basis;
    std::vector<double> inertia_min,inertia_max,normalized_time_weights,sample_times_s;
    std::string basis{"time_weighted_mean_of_URDF_path_mass_matrices"};
    bool measured{false};
};

// Trapezoidal time weights describe the chosen nominal reference, not new
// measured inertia or a worst-case all-pose modal guarantee. The convex mean
// remains SPD and the original per-axis diagonal inertia range is preserved.
// Times may be nonuniform; no trajectory duration or task budget is changed.
inline UrdfPathInertiaBasis time_weighted_urdf_path_inertia(
    const std::vector<std::vector<std::vector<double>>>& matrices,const std::vector<double>& sample_times_s) {
    if(matrices.empty() || matrices.size()!=sample_times_s.size())
        throw std::invalid_argument("URDF path inertia requires one timestamp for every complete mass matrix");
    for(std::size_t k=0;k<sample_times_s.size();++k)
        if(!std::isfinite(sample_times_s[k]) || (k && sample_times_s[k]<=sample_times_s[k-1]))
            throw std::invalid_argument("URDF path inertia sample times must be finite and strictly increasing");
    UrdfPathInertiaBasis out;out.sample_times_s=sample_times_s;
    out.normalized_time_weights.assign(matrices.size(),matrices.size()==1?1.:0.);
    if(matrices.size()>1) {
        const double duration=sample_times_s.back()-sample_times_s.front();
        if(!std::isfinite(duration) || duration<=0)throw std::invalid_argument("URDF path inertia time span is invalid");
        for(std::size_t k=1;k<matrices.size();++k) {
            const double half_interval=.5*((sample_times_s[k]-sample_times_s[k-1])/duration);
            out.normalized_time_weights[k-1]+=half_interval;out.normalized_time_weights[k]+=half_interval;
        }
    }
    const auto n=matrices.front().size();Eigen::MatrixXd mean=Eigen::MatrixXd::Zero(n,n);
    out.inertia_min.assign(n,std::numeric_limits<double>::infinity());out.inertia_max.assign(n,0);
    for(std::size_t k=0;k<matrices.size();++k) {
        const auto M=urdf_nominal_detail::checked_mass_matrix(matrices[k]);
        if(matrices[k].size()!=n)throw std::invalid_argument("URDF path mass matrix joint mapping changed");
        mean+=out.normalized_time_weights[k]*M;
        for(std::size_t i=0;i<n;++i) {
            out.inertia_min[i]=std::min(out.inertia_min[i],M(i,i));out.inertia_max[i]=std::max(out.inertia_max[i],M(i,i));
        }
    }
    out.mass_matrix_basis.assign(n,std::vector<double>(n));
    for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)out.mass_matrix_basis[i][j]=mean(i,j);
    (void)urdf_nominal_detail::checked_mass_matrix(out.mass_matrix_basis);
    return out;
}

struct UrdfNominalGainDesign {
    std::vector<double> unconstrained_kp,unconstrained_kd,inertia_basis;
    std::vector<std::vector<double>> mass_matrix_basis;
    // The full local modal damping design is diagnostic; MIT implements its
    // positive diagonal projection, not the cross-axis velocity terms.
    std::vector<std::vector<double>> full_damping_matrix;
    std::string basis{"single_pose_URDF_mass_matrix"};
    std::vector<double> inertia_min,inertia_max,normalized_time_weights{1.},sample_times_s{0.};
    std::string damping_design_method{"double_diagonalization_diagonal_projection"};
    std::string damping_interface_projection{"retain_diagonal_for_independent_mit_derivative_channels"};
    double design_frequency_hz{2},damping_ratio{.7};
    UrdfNominalModalDiagnostics modal_diagnostics;
    bool measured{false};
    std::string source{"urdf_geometry_mass_matrix_only_nominal_response_design_double_diagonalization_diagonal_projection"};
};
// R_ii=sum_j |M_ij| gives R-M positive semidefinite for symmetric M.
// K=omega^2 R makes the design frequency a lower bound on undamped ideal
// model frequencies, rather than forcing all coupled modes to that frequency.
// Double diagonalization: K*Phi=M*Phi*Omega^2, Phi^T*M*Phi=I, then
// D_full=M*Phi*(2*zeta*Omega)*Phi^T*M assigns the requested local modal damping.
// See Albu-Schaeffer et al., "A Passivity Based Cartesian Impedance Controller
// for Flexible Joint Robots - Part II", ICRA 2004, IV.B Eq.35:
// https://www.robotic.dlr.de/fileadmin/robotic/ott/papers/icra2004_II.pdf
// Independent MIT derivative channels implement diag(D_full). This remains
// positive damping, but generally loses modal decoupling and exact zeta;
// the reported modal damping ratios therefore describe projections only.
// Frequency/zeta are explicit engineering parameters, not URDF measurements.
// Actuator delay, filtering, friction and flexible modes remain unmeasured.
// No old gains, path, artificial URDF limits or protocol range enters here.
inline UrdfNominalGainDesign nominal_urdf_gain_design(
    const std::vector<std::vector<double>>& mass_matrix,double design_frequency_hz=2,double damping_ratio=.7) {
    const auto n=mass_matrix.size();
    if(!n || !std::isfinite(design_frequency_hz) || design_frequency_hz<=0 ||
       !std::isfinite(damping_ratio) || damping_ratio<=0)
        throw std::invalid_argument("URDF nominal gain design requires positive response frequency and damping targets");
    const auto M=urdf_nominal_detail::checked_mass_matrix(mass_matrix);
    UrdfNominalGainDesign out;out.mass_matrix_basis=mass_matrix;
    out.design_frequency_hz=design_frequency_hz;out.damping_ratio=damping_ratio;
    const double omega=2*3.14159265358979323846*design_frequency_hz;
    for(std::size_t i=0;i<n;++i) {
        double row=0;for(double value:mass_matrix[i])row+=std::abs(value);
        const double kp=omega*omega*row;
        if(!std::isfinite(kp))throw std::invalid_argument("URDF nominal gain overflow; repair inertial data");
        out.inertia_basis.push_back(row);out.unconstrained_kp.push_back(kp);
        out.inertia_min.push_back(M(i,i));out.inertia_max.push_back(M(i,i));
    }
    const auto damping=nominal_urdf_damping_design(mass_matrix,out.unconstrained_kp,damping_ratio);
    out.unconstrained_kd=damping.unconstrained_kd;out.full_damping_matrix=damping.full_damping_matrix;
    out.modal_diagnostics=damping.modal_diagnostics;
    return out;
}

inline UrdfNominalGainDesign nominal_urdf_path_gain_design(
    const std::vector<std::vector<std::vector<double>>>& matrices,const std::vector<double>& sample_times_s,
    double design_frequency_hz=2,double damping_ratio=.7) {
    const auto reference=time_weighted_urdf_path_inertia(matrices,sample_times_s);
    auto out=nominal_urdf_gain_design(reference.mass_matrix_basis,design_frequency_hz,damping_ratio);
    out.basis=reference.basis;out.inertia_min=reference.inertia_min;out.inertia_max=reference.inertia_max;
    out.normalized_time_weights=reference.normalized_time_weights;out.sample_times_s=reference.sample_times_s;
    out.source="urdf_geometry_time_weighted_path_mass_matrix_nominal_response_design_double_diagonalization_diagonal_projection";
    return out;
}
} // namespace serial_arm::servo_tuning
