#include <alica_tests/plans/ValueMappingPlanTestPlan.h>
#include <any>
#include <optional>
#include <string>

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
        if (bb.get<int64_t>("inputInt") != -8 || bb.get<uint64_t>("inputUint") != 19 || bb.get<bool>("inputBool") != true ||
                bb.get<std::string>("inputString") != "test" || bb.get<double>("inputDouble") != 12.6) {
            std::stringstream ss;
            ss << "ValueMappingCondition: One or more of the mapped values are wrong:\n"
               << "key: mappedIntValue, value: " << bb.get<int64_t>("mappedIntValue") << ", expected: -3\n"
               << "key: mappedUintValue, value: " << bb.get<uint64_t>("mappedUintValue") << ", expected: 17\n"
               << "key: mappedBoolValue, value: " << bb.get<bool>("mappedBoolValue") << ", expected: true\n"
               << "key: mappedStringValue, value: " << bb.get<std::string>("mappedStringValue") << ", expected: test\n"
               << "key: mappedDoubleValue, value: " << bb.get<double>("mappedDoubleValue") << ", expected: 3.7\n";
            LockedBlackboardRW gb(*getGlobalBlackboard());
            gb.set<std::any>("testError", std::optional<std::string>{ss.str()});
        }
    } catch (const BlackboardException& e) {
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("testError", std::string(e.what()));
    }
}
} // namespace alica
