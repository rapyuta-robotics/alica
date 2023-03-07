#include <alica_tests/plans/TestBlackboardPlan.h>

namespace alica
{
TestBlackboardPlan::TestBlackboardPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

bool ValueMappingConditionTestState2BlackboardTestSuccessState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    return bb.get<bool>("inputBool") == true && bb.get<int64_t>("inputInt") == 17 && bb.get<std::string>("inputString") == "test" &&
           bb.get<double>("inputDouble") == 5.2 && bb.get<uint64_t>("inputUint") == 1;
}
} // namespace alica
