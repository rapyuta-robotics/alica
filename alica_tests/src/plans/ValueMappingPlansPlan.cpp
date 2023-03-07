#include <alica_tests/plans/ValueMappingPlansPlan.h>

namespace alica
{
ValueMappingPlansPlan::ValueMappingPlansPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

bool ValueMappingPlansState2ValueMappingPlansSuccessState(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    return bb.get<bool>("inputBool") == true && bb.get<int64_t>("inputInt") == 16 && bb.get<std::string>("inputString") == "test" &&
           bb.get<double>("inputDouble") == 0.2 && bb.get<uint64_t>("inputUint") == 13;
}
} // namespace alica
