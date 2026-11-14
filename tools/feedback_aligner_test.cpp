#include "serial_arm/interaction/runtime/feedback_aligner.hpp"
#include <cassert>
#include <cmath>
#include <iostream>
using namespace serial_arm;
int main(){
    FeedbackAligner aligner;
    JointState first{{1,2},{10,20},{100,200}};
    auto no_overlap=aligner.update(first,{100,110},{1,1},110,100,30);
    assert(!no_overlap.valid);
    JointState second{{3,4},{30,40},{300,400}};
    auto aligned=aligner.update(second,{120,130},{1,1},130,100,30);
    assert(aligned.valid && aligned.fresh && aligned.timestamp_ns==120);
    assert(aligned.state.pos[0]==3 && std::abs(aligned.state.pos[1]-3)<1e-12);
    assert(std::abs(aligned.state.vel[1]-30)<1e-12 && std::abs(aligned.state.tor[1]-300)<1e-12);
    auto repeated=aligner.update(second,{120,130},{1,1},135,100,30);
    assert(repeated.valid && !repeated.fresh);
    auto stale=aligner.update(second,{120,130},{1,1},300,100,30);
    assert(!stale.valid);
    auto no_timing=aligner.update(second,{300,300},{1,0},300,100,30);
    assert(!no_timing.valid);
    std::cout<<"feedback alignment passed\n";
}
