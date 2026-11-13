#include "serial_arm/dynamics/urdf_nominal_gain_design.hpp"
#include <Eigen/Eigenvalues>
#include <iostream>

using namespace serial_arm::servo_tuning;
namespace {
constexpr double pi=3.14159265358979323846;
void check(bool condition,const char* message){if(!condition)throw std::runtime_error(message);}
void coupled_modes() {
    const std::vector<std::vector<double>> input{{.4,-.12,.04},{-.12,.2,-.03},{.04,-.03,.08}};
    const auto design=nominal_urdf_gain_design(input);
    const std::vector<double> expected{.56,.35,.15};
    for(std::size_t j=0;j<3;++j)check(std::abs(design.inertia_basis[j]-expected[j])<1e-12,"absolute row sums omit coupling or signs");
    Eigen::Matrix3d M,K,D,R,full_D;
    K.setZero();D.setZero();R.setZero();
    for(int i=0;i<3;++i) {
        for(int j=0;j<3;++j){M(i,j)=input[i][j];full_D(i,j)=design.full_damping_matrix[i][j];}
        K(i,i)=design.unconstrained_kp[i];D(i,i)=design.unconstrained_kd[i];R(i,i)=design.inertia_basis[i];
    }
    check(Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(R-M).eigenvalues().minCoeff()>-1e-12,
        "coupled inertia majorant is not positive semidefinite");
    const Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::Matrix3d> modes(K,M);
    const double omega=2*3.14159265358979323846*2;
    for(int i=0;i<3;++i) {
        const auto v=modes.eigenvectors().col(i);const double w=std::sqrt(modes.eigenvalues()[i]);
        check(w>=omega-1e-10,"nominal coupled modes violate the retained stiffness frequency bound");
        check(std::abs(v.dot(full_D*v)/(2*w)-.7)<1e-10,
            "double diagonalization failed to assign full-matrix local modal damping");
        check(std::abs(design.modal_diagnostics.nominal_mode_frequency_hz[i]-w/(2*pi))<1e-10 &&
              std::abs(design.modal_diagnostics.nominal_mode_damping_ratio[i]-v.dot(D*v)/(2*w))<1e-10,
            "reported model modes substitute full-matrix damping for the actual MIT diagonal projection");
        check(design.unconstrained_kd[i]>0 && design.unconstrained_kd[i]==full_D(i,i),
            "MIT derivative gain is not the positive diagonal projection of the full design");
    }
    check(Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d>(full_D).eigenvalues().minCoeff()>0 &&
          (modes.eigenvectors().transpose()*full_D*modes.eigenvectors()-
              (1.4*modes.eigenvalues().array().sqrt()).matrix().asDiagonal().toDenseMatrix()).norm()<1e-10,
        "full damping is not positive definite or introduces cross-modal damping");
    check(!design.modal_diagnostics.damping_projection_is_exact,
        "diagonal MIT projection was falsely declared to preserve exact modal decoupling");
    const double old_maximum=.7*std::sqrt(modes.eigenvalues().maxCoeff())/omega;
    check(*std::max_element(design.modal_diagnostics.nominal_mode_damping_ratio.begin(),
          design.modal_diagnostics.nominal_mode_damping_ratio.end())<old_maximum,
        "coupled example still uses stiffness-proportional damping with its fast-mode overdamping");
    check(design.modal_diagnostics.nominal_mode_frequency_hz.back()>2.5,
        "the 2Hz lower design bound was incorrectly reported as every coupled mode frequency");
}
void encoded_and_zero_stiffness_diagnostics() {
    const std::vector<std::vector<double>> M{{.4,.1},{.1,.2}};
    const auto design=nominal_urdf_gain_design(M);
    auto p=design.unconstrained_kp,d=design.unconstrained_kd;
    p[0]*=.4;d[1]*=.25;
    const auto encoded=ideal_urdf_modal_diagnostics(M,p,d);
    check(!encoded.damping_projection_is_exact,
        "unequal protocol clipping falsely retained the exact proportional damping interpretation");
    check(encoded.nominal_mode_frequency_hz.front()<design.modal_diagnostics.nominal_mode_frequency_hz.front(),
        "encoded-gain diagnostics reused the unclipped model mode frequencies");
    check(std::abs(encoded.nominal_mode_damping_ratio.front()-design.modal_diagnostics.nominal_mode_damping_ratio.front())>.01,
        "encoded-gain diagnostics reused the unclipped modal damping");
    const auto drag=ideal_urdf_modal_diagnostics(M,{0,0},design.unconstrained_kd);
    check(drag.nominal_mode_frequency_hz==std::vector<double>({0,0}) &&
          std::all_of(drag.nominal_mode_damping_ratio.begin(),drag.nominal_mode_damping_ratio.end(),
              [](double zeta){return std::isnan(zeta);}),
        "zero-stiffness drag was assigned a restoring bandwidth or fabricated zero modal damping ratio");
    bool rejected=false;
    try{(void)ideal_urdf_modal_diagnostics(M,{1},{1});}catch(const std::invalid_argument&){rejected=true;}
    check(rejected,"partial gain groups entered coupled modal diagnostics");
}
void dimensions_scale_and_mapping() {
    for(const std::size_t n:{1U,3U,7U}) {
        std::vector<std::vector<double>> M(n,std::vector<double>(n,0));
        for(std::size_t j=0;j<n;++j)M[j][j]=.001*std::pow(3,j);
        const auto original=nominal_urdf_gain_design(M);
        for(auto& row:M)for(auto& value:row)value*=5;
        const auto scaled=nominal_urdf_gain_design(M);
        check(scaled.unconstrained_kp.size()==n,"nominal design is tied to a particular robot joint count");
        for(std::size_t j=0;j<n;++j)check(std::abs(scaled.unconstrained_kp[j]-5*original.unconstrained_kp[j])<1e-10 &&
            std::abs(scaled.unconstrained_kd[j]-5*original.unconstrained_kd[j])<1e-10,
            "changing inertial scale does not proportionally change both nominal gains");
        for(std::size_t j=0;j<n;++j)check(std::abs(scaled.modal_diagnostics.nominal_mode_frequency_hz[j]-2)<1e-10 &&
            std::abs(scaled.modal_diagnostics.nominal_mode_damping_ratio[j]-.7)<1e-10,
            "uncoupled ideal mode diagnostics depend on inertia units or joint count");
    }
    const auto a=nominal_urdf_gain_design({{.4,.1},{.1,.2}});
    const auto b=nominal_urdf_gain_design({{.2,.1},{.1,.4}});
    check(std::abs(a.unconstrained_kp[0]-b.unconstrained_kp[1])<1e-12 &&
          std::abs(a.unconstrained_kd[1]-b.unconstrained_kd[0])<1e-12,
        "joint mapping permutation changes physical design");
    for(std::size_t i=0;i<2;++i)check(std::abs(a.modal_diagnostics.nominal_mode_frequency_hz[i]-b.modal_diagnostics.nominal_mode_frequency_hz[i])<1e-10 &&
        std::abs(a.modal_diagnostics.nominal_mode_damping_ratio[i]-b.modal_diagnostics.nominal_mode_damping_ratio[i])<1e-10,
        "joint mapping permutation changes the ideal model modes");
    const auto slower=nominal_urdf_gain_design({{.4,.1},{.1,.2}},1,.7);
    check(std::abs(slower.unconstrained_kp[0]-a.unconstrained_kp[0]/4)<1e-12 &&
          std::abs(slower.unconstrained_kd[0]-a.unconstrained_kd[0]/2)<1e-12,
        "explicit response design cannot be distinguished from URDF physical prior");
}
void coupled_dimensions_scale_and_permutation() {
    for(const std::size_t n:{1U,3U,7U}) {
        Eigen::MatrixXd B(n,n);
        for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j)
            B(i,j)=std::sin(static_cast<double>((i+1)*(j+2)))*(.2+.05*i);
        Eigen::MatrixXd M=B*B.transpose();M.diagonal().array()+=.03;
        std::vector<std::vector<double>> input(n,std::vector<double>(n)),scaled=input,permuted=input;
        for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j) {
            input[i][j]=M(i,j);scaled[i][j]=5*M(i,j);permuted[n-i-1][n-j-1]=M(i,j);
        }
        const auto a=nominal_urdf_gain_design(input),b=nominal_urdf_gain_design(scaled),p=nominal_urdf_gain_design(permuted);
        Eigen::VectorXd P(n),D(n);Eigen::MatrixXd full(n,n);
        for(std::size_t i=0;i<n;++i) {
            P(i)=a.unconstrained_kp[i];D(i)=a.unconstrained_kd[i];
            check(std::abs(b.unconstrained_kp[i]-5*P(i))<1e-10 && std::abs(b.unconstrained_kd[i]-5*D(i))<1e-10,
                "coupled inertia scaling did not scale nominal P and projected D together");
            check(std::abs(p.unconstrained_kp[n-i-1]-P(i))<1e-10 && std::abs(p.unconstrained_kd[n-i-1]-D(i))<1e-10,
                "coupled design depends on joint identity or permutation");
            check(D(i)>0,"diagonal projection lost positive ideal damping");
            for(std::size_t j=0;j<n;++j) {
                full(i,j)=a.full_damping_matrix[i][j];
                check(std::abs(b.full_damping_matrix[i][j]-5*full(i,j))<1e-10 &&
                      std::abs(p.full_damping_matrix[n-i-1][n-j-1]-full(i,j))<1e-10,
                    "full double-diagonalized damping is not equivariant under scale or mapping");
            }
        }
        const Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::MatrixXd> modes(P.asDiagonal().toDenseMatrix(),M);
        const Eigen::MatrixXd actual=modes.eigenvectors().transpose()*full*modes.eigenvectors();
        const Eigen::MatrixXd desired=(1.4*modes.eigenvalues().array().sqrt()).matrix().asDiagonal();
        check(full.isApprox(full.transpose(),1e-12) && Eigen::LLT<Eigen::MatrixXd>(full).info()==Eigen::Success &&
              actual.isApprox(desired,1e-10),"arbitrary coupled model did not obtain positive full-matrix exact modal damping");
        // diag(D_full)>0 gives d/dt(.5*v^T*M*v+.5*q^T*K*q)=-v^T*D*v<0
        // in this ideal continuous, fixed-pose model. No inner-loop model is assumed.
        const Eigen::VectorXd velocity=Eigen::VectorXd::Ones(n);
        check(velocity.dot(D.asDiagonal()*velocity)>0,"projected ideal damping does not dissipate kinetic energy");
        check(!a.measured && a.source.find("double_diagonalization_diagonal_projection")!=std::string::npos,
            "new design was reported as measured or retained the obsolete damping source");
    }
    const auto strong=nominal_urdf_gain_design({{1,.99},{.99,1}});
    check(!strong.modal_diagnostics.damping_projection_is_exact ||
          std::abs(strong.modal_diagnostics.nominal_mode_damping_ratio.front()-.7)>.01,
        "a diagonal interface projection was silently advertised as all modes achieving the requested ratio");
    check(*std::max_element(strong.modal_diagnostics.nominal_mode_damping_ratio.begin(),
          strong.modal_diagnostics.nominal_mode_damping_ratio.end())>1,
        "high coupling diagnostics hid the remaining model overdamping after diagonal projection");
}
void final_complete_stiffness_is_the_damping_basis() {
    const std::vector<std::vector<double>> matrix{{.4,-.12,.04},{-.12,.2,-.03},{.04,-.03,.08}};
    const auto nominal=nominal_urdf_gain_design(matrix);
    const std::vector<double> final_P{3,17,6};
    const auto damping=nominal_urdf_damping_design(matrix,final_P,.7);
    check(damping.stiffness_basis==final_P && damping.mass_matrix_basis==matrix && !damping.measured,
        "damping design silently reconstructed the earlier row-sum stiffness or changed its explicit model basis");
    Eigen::Matrix3d M,full;
    for(int i=0;i<3;++i)for(int j=0;j<3;++j){M(i,j)=matrix[i][j];full(i,j)=damping.full_damping_matrix[i][j];}
    const Eigen::Map<const Eigen::VectorXd> P(final_P.data(),3);
    const Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::Matrix3d> modes(P.asDiagonal().toDenseMatrix(),M);
    const Eigen::Matrix3d modal=modes.eigenvectors().transpose()*full*modes.eigenvectors();
    check((modal-(1.4*modes.eigenvalues().array().sqrt()).matrix().asDiagonal().toDenseMatrix()).norm()<1e-10,
        "given-final-K double diagonalization used another stiffness group");
    check(std::abs(damping.unconstrained_kd[0]-nominal.unconstrained_kd[0])>.1,
        "a materially changed final complete P left D tied to the earlier nominal recommendation");
    const auto scalar=nominal_urdf_damping_design({{4,0},{0,1}},{9,16});
    check(std::abs(scalar.unconstrained_kd[0]-1.4*std::sqrt(4*9))<1e-10 &&
          std::abs(scalar.unconstrained_kd[1]-1.4*std::sqrt(16))<1e-10,
        "uncoupled damping does not reduce to the scalar formula with the supplied final P");
    for(const auto& invalid:std::vector<std::vector<double>>{{1},{0,1,1},{1,-1,1},{1,1,std::numeric_limits<double>::infinity()}}) {
        bool rejected=false;try{(void)nominal_urdf_damping_design(matrix,invalid);}catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"partial/nonpositive final stiffness was silently filled with a nominal replacement");
    }
}

void weighted_path_basis_and_reversed_inertia() {
    const std::vector<std::vector<double>> a{{4,0},{0,1}},b{{1,0},{0,4}},c{{2,0},{0,8}};
    const auto equal=nominal_urdf_path_gain_design({a,b,a},{0,1,2});
    const double omega=4*pi;
    check(std::abs(equal.mass_matrix_basis[0][0]-2.5)<1e-12 && std::abs(equal.mass_matrix_basis[1][1]-2.5)<1e-12 &&
          equal.inertia_min==std::vector<double>({1,1}) && equal.inertia_max==std::vector<double>({4,4}),
        "reversed posture inertias were replaced with coordinatewise maxima instead of one coherent mean model");
    for(std::size_t i=0;i<2;++i) {
        check(std::abs(equal.unconstrained_kp[i]-2.5*omega*omega)<1e-10 &&
              std::abs(equal.unconstrained_kd[i]-1.4*omega*2.5)<1e-10,
            "path stiffness and damping did not share the same weighted matrix");
        check(std::abs(equal.modal_diagnostics.nominal_mode_damping_ratio[i]-.7)<1e-10,
            "coherent diagonal mean model did not retain its specified nominal damping");
    }
    check(equal.basis=="time_weighted_mean_of_URDF_path_mass_matrices" && !equal.measured &&
          equal.source.find("time_weighted_path_mass_matrix")!=std::string::npos,
        "path average was mislabeled as the single-pose basis or measured dynamics");
    const auto uneven=time_weighted_urdf_path_inertia({a,b,c},{0,1,5});
    const std::vector<double> expected_weights{.1,.5,.4};
    for(std::size_t k=0;k<3;++k)check(std::abs(uneven.normalized_time_weights[k]-expected_weights[k])<1e-12,
        "nonuniform sample intervals were replaced with equal matrix counts");
    check(std::abs(uneven.mass_matrix_basis[0][0]-1.7)<1e-12 && std::abs(uneven.mass_matrix_basis[1][1]-5.3)<1e-12,
        "nonuniform path time weighting is incorrect");
    check(uneven.sample_times_s==std::vector<double>({0,1,5}),"physical sample times were retimed or changed by nominal design");
    const auto shifted=time_weighted_urdf_path_inertia({a,b,c},{9,10,14});
    const auto reversed=time_weighted_urdf_path_inertia({c,b,a},{-5,-1,0});
    for(std::size_t i=0;i<2;++i)for(std::size_t j=0;j<2;++j)
        check(std::abs(shifted.mass_matrix_basis[i][j]-uneven.mass_matrix_basis[i][j])<1e-12 &&
              std::abs(reversed.mass_matrix_basis[i][j]-uneven.mass_matrix_basis[i][j])<1e-12,
            "time origin or reversing the same timed path changed its mean mechanical reference");
    const auto single=nominal_urdf_path_gain_design({a},{3});const auto original=nominal_urdf_gain_design(a);
    check(single.mass_matrix_basis==original.mass_matrix_basis && single.normalized_time_weights==std::vector<double>({1}) &&
          single.unconstrained_kp==original.unconstrained_kp && single.unconstrained_kd==original.unconstrained_kd,
        "one-pose path is not compatible with the original single-pose design interface");
}

void weighted_path_dimensions_and_mapping() {
    for(const std::size_t n:{1U,3U,7U}) {
        std::vector<std::vector<std::vector<double>>> matrices(2,std::vector<std::vector<double>>(n,std::vector<double>(n)));
        for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j) {
            // SPD diagonal plus a rank-one coupling, then a different positive
            // diagonal congruence in the second pose.
            const double value=(i==j?.1*(i+1):0)+.02*(i+1)*(j+1);
            matrices[0][i][j]=value;matrices[1][i][j]=value*(1+.2*i)*(1+.2*j);
        }
        auto scaled=matrices,permuted=matrices;
        for(std::size_t k=0;k<2;++k)for(std::size_t i=0;i<n;++i)for(std::size_t j=0;j<n;++j) {
            scaled[k][i][j]*=5;permuted[k][n-i-1][n-j-1]=matrices[k][i][j];
        }
        const auto a=nominal_urdf_path_gain_design(matrices,{0,2}),b=nominal_urdf_path_gain_design(scaled,{0,2}),
            p=nominal_urdf_path_gain_design(permuted,{0,2});
        for(std::size_t i=0;i<n;++i) {
            check(std::abs(b.unconstrained_kp[i]-5*a.unconstrained_kp[i])<1e-9 &&
                  std::abs(b.unconstrained_kd[i]-5*a.unconstrained_kd[i])<1e-9,
                "time-weighted design did not respect inertial scaling for an arbitrary arm size");
            check(std::abs(p.unconstrained_kp[n-i-1]-a.unconstrained_kp[i])<1e-9 &&
                  std::abs(p.unconstrained_kd[n-i-1]-a.unconstrained_kd[i])<1e-9 &&
                  p.inertia_min[n-i-1]==a.inertia_min[i] && p.inertia_max[n-i-1]==a.inertia_max[i],
                "path model reference or preserved inertia ranges depend on joint identity/permutation");
        }
    }
    const std::vector<std::vector<std::vector<double>>> matrices{{{1}},{{2}}};
    for(const auto& times:std::vector<std::vector<double>>{{},{0},{0,0},{1,0},{0,std::numeric_limits<double>::quiet_NaN()}}) {
        bool rejected=false;try{(void)time_weighted_urdf_path_inertia(matrices,times);}catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"missing/reversed/nonfinite path timestamps silently received equal-weight fallback");
    }
    for(const auto& invalid:std::vector<std::vector<std::vector<std::vector<double>>>>{
        {},{{{1}},{{0}}},{{{1}},{{1,0},{0,1}}}}) {
        bool rejected=false;try{(void)time_weighted_urdf_path_inertia(invalid,invalid.empty()?std::vector<double>{}:std::vector<double>{0,1});}
        catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"invalid inertia or changed mapping in one path sample was averaged into a valid design");
    }
}
void invalid_model() {
    for(const auto& matrix:std::vector<std::vector<std::vector<double>>>{{},{{0}},{{1,2},{2,1}},{{1,.1},{.2,1}},{{1,0},{0}}}) {
        bool rejected=false;try{(void)nominal_urdf_gain_design(matrix);}catch(const std::invalid_argument&){rejected=true;}
        check(rejected,"invalid or unmapped inertia silently received fallback gains");
    }
}
}
int main(){try{coupled_modes();encoded_and_zero_stiffness_diagnostics();dimensions_scale_and_mapping();coupled_dimensions_scale_and_permutation();
    final_complete_stiffness_is_the_damping_basis();weighted_path_basis_and_reversed_inertia();weighted_path_dimensions_and_mapping();invalid_model();
    std::cout<<"URDF nominal design: final-complete-K double diagonalization, time-weighted path reference/ranges, 1/3/7-axis mapping/scale, positive MIT projection and invalid-model rejection PASS\n";
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
