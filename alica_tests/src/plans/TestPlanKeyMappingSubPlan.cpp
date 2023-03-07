#include <alica_tests/plans/TestPlanKeyMappingSubPlan.h>

namespace alica
{
TestPlanKeyMappingSubPlan::TestPlanKeyMappingSubPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

bool TestPlanKeyMappingSubPlanEntryState2SuccessState(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    LockedBlackboardRO bb(*input);
    return bb.get<int64_t>("value") == 5;
}

} // namespace alica
