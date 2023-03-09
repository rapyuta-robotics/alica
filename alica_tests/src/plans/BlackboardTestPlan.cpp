#include <alica_tests/plans/BlackboardTestPlan.h>

namespace alica
{
BlackboardTestPlan::BlackboardTestPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void BlackboardTestPlan::onInit()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set<std::optional<std::string>>("testError", std::optional<std::string>());
}

bool ValueMappingCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO bb(*input);
    if (bb.get<int64_t>("mappedIntValue") == -3 && bb.get<uint64_t>("mappedUintValue") == 17 && bb.get<bool>("mappedBoolValue") == true &&
            bb.get<std::string>("mappedStringValue") == "test" && bb.get<double>("mappedDoubleValue") == 3.7) {
        return true;
    } else {
        std::stringstream ss;
        ss << "ValueMappingCondition: One or more of the mapped values are wrong:\n"
           << "key: mappedIntValue, value: " << bb.get<int64_t>("mappedIntValue") << ", expected: -3\n"
           << "key: mappedUintValue, value: " << bb.get<uint64_t>("mappedUintValue") << ", expected: 17\n"
           << "key: mappedBoolValue, value: " << bb.get<bool>("mappedBoolValue") << ", expected: true\n"
           << "key: mappedStringValue, value: " << bb.get<std::string>("mappedStringValue") << ", expected: test\n"
           << "key: mappedDoubleValue, value: " << bb.get<double>("mappedDoubleValue") << ", expected: 3.7\n";
        LockedBlackboardRW globalBlackboard(*(const_cast<Blackboard*>(gb)));
        globalBlackboard.set("testError", ss.str());
    }
    return false;
}
} // namespace alica
