#include <alica_tests/plans/BlackboardTestPlan.h>

namespace alica
{
BlackboardTestPlan::BlackboardTestPlan(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

bool ValueMappingCondition(const Blackboard* input, const RunningPlan* rp, const Blackboard* gb)
{
    LockedBlackboardRO bb(*input);
    if (bb.get<int>("mappedIntValue") == -3 && bb.get<uint64_t>("mappedUintValue") == 17 && bb.get<bool>("mappedBoolValue") == true &&
            bb.get<std::string>("mappedStringValue") == "test" && bb.get<double>("mappedDoubleValue") == 3.7) {
        return true;
    } else {
        Logging::logError("ValueMappingCondition") << "ValueMappingCondition: One or more of the mapped values are wrong:\n"
                                                   << "key: mappedIntValue, value: " << bb.get<int64_t>("mappedIntValue") << ", expected: -3\n"
                                                   << "key: mappedUintValue, value: " << bb.get<uint64_t>("mappedUintValue") << ", expected: 17\n"
                                                   << "key: mappedBoolValue, value: " << bb.get<bool>("mappedBoolValue") << ", expected: true\n"
                                                   << "key: mappedStringValue, value: " << bb.get<std::string>("mappedStringValue") << ", expected: test\n"
                                                   << "key: mappedDoubleValue, value: " << bb.get<double>("mappedDoubleValue") << ", expected: 3.7\n";
    }
    return false;
}
} // namespace alica
