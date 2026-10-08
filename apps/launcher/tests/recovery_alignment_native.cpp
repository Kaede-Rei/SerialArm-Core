#include "serial_arm/dynamics/calibration_recovery_alignment.hpp"
#include <cassert>
#include <iostream>
#include <vector>
int main() {
  using namespace serial_arm::calibration_recovery;
  const std::vector<double> start{0,0,0}, end{1,1,1};
  auto at_first=select_endpoint({0.03,0.01,0},{0,0,0},start,end,2);
  assert(at_first.valid && at_first.reverse_recorded_path && at_first.at_endpoint);
  auto at_last=select_endpoint({1.02,1,1},{0,0,0},start,end,2);
  assert(at_last.valid && !at_last.reverse_recorded_path && at_last.at_endpoint);
  auto far=select_endpoint({0.3,0.4,0.6},{0,0,0},start,end,2);
  assert(far.valid && !far.at_endpoint);
  auto high_velocity=select_endpoint({0,0,0},{0.06,0,0},start,end,2);
  assert(high_velocity.valid && !high_velocity.at_endpoint);
  auto stale=select_endpoint({0,0,0},{0,0,0},start,end,501);
  assert(!stale.valid);
  auto invalid=select_endpoint({0,0,0},{0,0,0},{0,0},end,2);
  assert(!invalid.valid);
  auto nan=select_endpoint({0,0,0},{0,0,0},start,end,0.0/0.0);
  assert(!nan.valid);
  std::cout << "7 endpoint-alignment cases passed\n";
}
