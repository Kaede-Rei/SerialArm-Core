#include "serial_arm/dynamics/calibration_recovery_alignment.hpp"
#include <iostream>
#include <stdexcept>

namespace recovery=serial_arm::calibration_recovery;
namespace {
void check(bool value,const char* message){if(!value)throw std::runtime_error(message);}
void continuing_reference_is_not_tracking_acceptance() {
    for(const std::size_t n:{1U,6U,8U}) {
        const std::vector<double> endpoint(n,1),actual(n,.7),velocity(n,.01),fresh(n,5);
        check(!recovery::select_endpoint(actual,velocity,endpoint,endpoint,5).at_endpoint,
            "cold manual entry lost the existing nearby endpoint requirement");
        auto continuation=recovery::automatic_endpoint_continuation(actual,endpoint,endpoint,endpoint,fresh,true);
        check(continuation.valid && continuation.reference_origin==endpoint,
            "a completed reference endpoint was blocked by actual tracking lag");
        check(actual==std::vector<double>(n,.7),"continuation changed actual feedback to manufacture alignment");
        const std::vector<double> accepted_after_hold(n,.8);
        continuation=recovery::automatic_endpoint_continuation(actual,accepted_after_hold,endpoint,endpoint,fresh,true);
        check(continuation.valid && continuation.reference_origin==accepted_after_hold,
            "automatic hold-to-path continuation jumped from the actual feedback instead of the accepted reference");
        for(int k=0;k<=100;++k) {
            const auto q=recovery::endpoint_bridge_reference(continuation.reference_origin,endpoint,k/100.0);
            for(double value:q)check(value>=.8 && value<=1,
                "the bounded reference bridge exceeded the current accepted reference/known taught endpoint envelope");
        }
        check(recovery::endpoint_bridge_reference(accepted_after_hold,endpoint,0)==accepted_after_hold &&
              recovery::endpoint_bridge_reference(accepted_after_hold,endpoint,1)==endpoint,
            "automatic bridge introduces a position reference step at either end");
    }
}
void continuity_is_evidence_not_a_global_bypass() {
    const std::vector<double> endpoint{1},actual{.9},accepted{1},fresh{5};
    check(recovery::emitted_endpoint_matches(accepted,endpoint),"actually emitted endpoint was not recognized");
    check(!recovery::emitted_endpoint_matches({.999},endpoint),"planned-but-unaccepted endpoint became a continuation credential");
    check(!recovery::automatic_endpoint_continuation(actual,accepted,{},endpoint,fresh,true).valid,
        "a cold task bypassed manual alignment without a completed endpoint record");
    check(!recovery::automatic_endpoint_continuation(actual,accepted,endpoint,{0},fresh,true).valid,
        "the continuation shortcut moved to the opposite demonstrated endpoint");
    check(!recovery::automatic_endpoint_continuation(actual,accepted,endpoint,endpoint,{60.001},true).valid,
        "stale feedback bypassed the unchanged fresh-feedback requirement");
    check(!recovery::automatic_endpoint_continuation(actual,accepted,endpoint,endpoint,fresh,false).valid,
        "a stopped or faulted robot entered automatic continuation");
    check(!recovery::automatic_endpoint_continuation(actual,{std::numeric_limits<double>::quiet_NaN()},endpoint,endpoint,fresh,true).valid,
        "unknown accepted reference was replaced with the actual position");
}
}
int main(){try{continuing_reference_is_not_tracking_acceptance();continuity_is_evidence_not_a_global_bypass();
    std::cout<<"automatic endpoint continuation: completed accepted reference, raw lag preservation, cold/fresh/Safety gates, bounded bridge PASS\n";
}catch(const std::exception& error){std::cerr<<error.what()<<'\n';return 1;}}
