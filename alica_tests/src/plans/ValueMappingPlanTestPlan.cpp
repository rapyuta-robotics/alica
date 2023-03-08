#include <alica_tests/plans/ValueMappingPlanTestPlan.h>

namespace alica
{
ValueMappingPlanTestPlan::ValueMappingPlanTestPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void ValueMappingPlanTestPlan::onInit()
{
    try {
        LockedBlackboardRO bb(*getBlackboard());
        assert(bb.get<int64_t>("inputInt") == -8 && "ValueMappingTestPlan: wrong inputInt value");
        assert(bb.get<uint64_t>("inputUint") == 19 && "ValueMappingTestPlan: wrong inputUint value");
        assert(bb.get<bool>("inputBool") == true && "ValueMappingTestPlan: wrong inputBool value");
        assert(bb.get<std::string>("inputString") == "test" && "ValueMappingTestPlan: wrong inputString value");
        assert(bb.get<double>("inputDouble") == 12.6 && "ValueMappingTestPlan: wrong inputDouble value");
    } catch (const BlackboardException& e) {
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("ValueMappingTestBehError", std::string(e.what()));
    }
}
} // namespace alica
