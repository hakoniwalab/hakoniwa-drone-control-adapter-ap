#pragma once
#include "hakoniwa/drone/control_adapter/allocation_feedback_policy.hpp"
namespace hakoniwa::drone::control_adapter {
class ApAllocationFeedbackPolicy final : public IAllocationFeedbackPolicy {
public:
    void reset() override;
    RateControlSaturation run(const AllocationFeedbackPolicyInput&) override;
};
}  // namespace hakoniwa::drone::control_adapter
