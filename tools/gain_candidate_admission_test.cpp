#include "serial_arm/dynamics/gain_candidate_admission.hpp"

#include <iostream>
#include <stdexcept>

using namespace serial_arm::servo_tuning;
namespace {
void check(bool value,const char* reason){if(!value)throw std::runtime_error(reason);}
void near(double a,double b,const char* reason){check(std::abs(a-b)<1e-10*std::max({1.0,std::abs(a),std::abs(b)}),reason);}
ModelAxisLimits limits(double budget=2){return {.04,5,20,budget,500,5,{0,0}};}

void real_failed_entry_is_rejected_as_a_complete_group() {
    // The real task requested a 533x stiffness increase while the already
    // accepted endpoint reference lagged actual feedback by 0.361639 rad.
    const Gains incumbent{{.3663003663003663},{.028083028083028084}};
    const Gains proposal{{195.23809523809524},{1.2881562881562882}};
    const auto current_copy=incumbent,proposal_copy=proposal;
    auto axis=limits();axis.grid={500.0/4095,5.0/4095};
    const auto result=admit_complete_gain_candidate(incumbent,proposal,{axis},
        {.13580608367919922},{.05},{-.36163902282714844},{.0073261260986328125});
    check(!result.allowed,"the real unmeasured high-P entry candidate was admitted");
    near(result.error_envelope_rad[0],.36163902282714844,"current accepted-minus-actual error was not included");
    check(result.proposal_PD_envelope_nm[0]>70 && result.incumbent_PD_envelope_nm[0]<.14,
        "the real PD demand change was not exposed in admission evidence");
    check(result.increase_PD_envelope_nm[0]>70 && result.feedback_effort_budget_nm[0]==2,
        "the increased demand or per-axis budget was not recorded");
    check(incumbent.kp==current_copy.kp && incumbent.kd==current_copy.kd &&
          proposal.kp==proposal_copy.kp && proposal.kd==proposal_copy.kd,
        "admission silently projected or changed a complete measured/proposed group");

    const Gains whole_old{{9.645909645909645,61.78266178266178,66.17826617826617,17.094017094017094,3.785103785103785,.3663003663003663},
        {.8595848595848596,4.920634920634921,4.783882783882784,.7081807081807082,.2087912087912088,.028083028083028084}};
    const Gains whole_new{{500,500,500,136.75213675213675,121.48962148962148,195.23809523809524},
        {2.432234432234432,5,5,3.9914529914529915,1.1807081807081807,1.2881562881562882}};
    std::vector<ModelAxisLimits> bounds(6,axis);
    const auto complete=admit_complete_gain_candidate(whole_old,whole_new,bounds,std::vector<double>(6,.01),
        std::vector<double>(6,.05),{.061417579650878906,-.006484985351562498,-.01335144042968749,
        -.0026702880859375017,-.04425048828125,-.36163902282714844},std::vector<double>(6,.0073261260986328125),
        {8,8,8,2,2,2});
    check(!complete.allowed && complete.axis_reasons.size()==6 && complete.proposal_PD_envelope_nm.back()>70,
        "the real complete six-axis group was split or its unsafe axis was hidden");
}

void general_counts_local_changes_and_explicit_budgets() {
    for(std::size_t count:{1U,2U,6U,8U}) {
        const Gains old{std::vector<double>(count,20),std::vector<double>(count,.4)};
        const Gains next{std::vector<double>(count,22),std::vector<double>(count,.45)};
        std::vector<ModelAxisLimits> bounds(count,limits());
        const auto result=admit_complete_gain_candidate(old,next,bounds,std::vector<double>(count,.02),
            std::vector<double>(count,.1),std::vector<double>(count,.01),std::vector<double>(count,-.05));
        check(result.allowed && result.axis_reasons.size()==count,"a reasonable generic local group was rejected");
        for(std::size_t axis=0;axis<count;++axis) {
            near(result.incumbent_PD_envelope_nm[axis],.44,"incumbent envelope differs from the recorded experiment");
            near(result.proposal_PD_envelope_nm[axis],.485,"candidate envelope was not computed on measured peaks");
            near(result.increase_PD_envelope_nm[axis],.045,"positive P/D changes were not accounted independently");
        }
        auto budget=std::vector<double>(count,2);budget.back()=.3;
        const auto blocked=admit_complete_gain_candidate(old,next,bounds,std::vector<double>(count,.02),
            std::vector<double>(count,.1),std::vector<double>(count,.01),std::vector<double>(count,-.05),budget);
        check(!blocked.allowed && blocked.feedback_effort_budget_nm.back()==.3,
            "a single axis's actual experiment budget did not reject the whole candidate");
    }
    auto protocol=limits();protocol.grid={500.0/4095,5.0/4095};
    const Gains old{{100*protocol.grid.kp_step},{200*protocol.grid.kd_step}};
    const Gains next{{101*protocol.grid.kp_step},{201*protocol.grid.kd_step}};
    check(admit_complete_gain_candidate(old,next,{protocol},{.02},{.05},{0},{0}).allowed,
        "one actually encodable protocol-code increase was rejected");
    check(!admit_complete_gain_candidate(old,{{12.25},{.25}},{protocol},{.02},{.05},{0},{0}).allowed,
        "admission silently encoded a different, unmeasured gain group");
}

void large_error_reductions_and_zero_stiffness() {
    const Gains old{{100},{.5}},lower{{50},{.25}};
    const auto reduction=admit_complete_gain_candidate(old,lower,{limits()},{.5},{1},{-.4},{-.5});
    check(reduction.allowed && reduction.incumbent_PD_envelope_nm[0]>2 && reduction.proposal_PD_envelope_nm[0]>2,
        "large existing tracking error became a new arrival gate for a demand-reducing candidate");
    check(reduction.increase_PD_envelope_nm[0]==0,"a componentwise gain reduction was reported as increased feedback demand");
    const auto trade=admit_complete_gain_candidate(old,{{80},{.6}},{limits()},{.5},{1},{0},{0});
    check(!trade.allowed && trade.proposal_PD_envelope_nm[0]<trade.incumbent_PD_envelope_nm[0],
        "a P/D cancellation was credited as effort room when the old group already exceeded the experiment budget");
    check(!admit_complete_gain_candidate({{10},{1}},{{11},{.9}},{limits()},{.1},{1},{0},{0}).allowed,
        "decreased damping was used to conceal a new stiffness demand at a full budget");

    const auto drag=admit_complete_gain_candidate({{0},{.4}},{{0},{.5}},{limits()},{.4},{.2},{-.3},{-.1});
    check(drag.allowed && drag.error_envelope_rad[0]==.4 && drag.proposal_PD_envelope_nm[0]==.1,
        "zero-stiffness drag was blocked by a position-accuracy requirement");
    const auto quiet=admit_complete_gain_candidate({{0},{.4}},{{0},{.5}},{limits()},{0},{0},{0},{0});
    check(quiet.allowed && quiet.error_envelope_rad[0]==0 && quiet.proposal_PD_envelope_nm[0]==0,
        "zero measured/fresh error invented motion or caused a division failure");
}

void incomplete_nonfinite_and_protocol_evidence_is_rejected() {
    const Gains old{{20},{.4}},next{{22},{.45}};
    const double nan=std::numeric_limits<double>::quiet_NaN();
    check(!admit_complete_gain_candidate(old,next,{limits()},{},{.1},{0},{0}).allowed,"missing measured error was defaulted to zero");
    check(!admit_complete_gain_candidate(old,next,{limits()},{.01},{.1},{},{0}).allowed,"missing fresh error was defaulted to zero");
    check(!admit_complete_gain_candidate(old,next,{limits()},{nan},{.1},{0},{0}).allowed,"nonfinite measured evidence was admitted");
    check(!admit_complete_gain_candidate(old,next,{limits()},{.01},{.1},{nan},{0}).allowed,"nonfinite fresh evidence was admitted");
    check(!admit_complete_gain_candidate(old,next,{limits()},{-.01},{.1},{0},{0}).allowed,"negative measured peak was admitted");
    check(!admit_complete_gain_candidate(old,next,{limits()},{.01},{.1},{0},{0},{nan}).allowed,"nonfinite feedback budget was admitted");
    check(!admit_complete_gain_candidate(old,{{500.0001},{.4}},{limits()},{.01},{.1},{0},{0}).allowed,"out-of-protocol P was clipped and admitted");
    check(!admit_complete_gain_candidate(old,{{20},{5.0001}},{limits()},{.01},{.1},{0},{0}).allowed,"out-of-protocol D was clipped and admitted");
    check(!admit_complete_gain_candidate(old,{{nan},{.4}},{limits()},{.01},{.1},{0},{0}).allowed,"nonfinite proposal was admitted");
    const auto missing=admit_complete_gain_candidate(old,next,{limits()},{},{.1},{0},{0});
    check(missing.proposal_PD_envelope_nm.empty(),"missing evidence produced fabricated numerical PD measurements");
}
}
int main() {
    try {
        real_failed_entry_is_rejected_as_a_complete_group();general_counts_local_changes_and_explicit_budgets();
        large_error_reductions_and_zero_stiffness();incomplete_nonfinite_and_protocol_evidence_is_rejected();
        std::cout<<"complete gain candidate admission: live/measured PD envelopes, actual fault regression, whole-group refusal, reductions, drag and generic dimensions PASS\n";
    }catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}
}
