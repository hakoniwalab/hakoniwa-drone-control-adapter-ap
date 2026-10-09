// SPDX-License-Identifier: GPL-3.0-or-later
#include "hakoniwa/drone/control_adapter/ap_allocation_feedback_policy.hpp"
#include <cmath>
namespace hakoniwa::drone::control_adapter {
namespace { AxisSaturationFlags flags(double v) { return {v > 1e-6, v < -1e-6}; } }
void ApAllocationFeedbackPolicy::reset() {}
RateControlSaturation ApAllocationFeedbackPolicy::run(const AllocationFeedbackPolicyInput& in)
{ return {flags(in.allocation_status.unallocated_torque_x), flags(in.allocation_status.unallocated_torque_y), flags(in.allocation_status.unallocated_torque_z)}; }
}  // namespace hakoniwa::drone::control_adapter
