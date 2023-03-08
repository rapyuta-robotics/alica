#include <alica_tests/behaviours/ValueMappingTestBeh.h>
#include <memory>
#include <sstream>

namespace alica
{
ValueMappingTestBeh::ValueMappingTestBeh(BehaviourContext& context)
        : BasicBehaviour(context)
{
}

void ValueMappingTestBeh::run()
{
    if (isSuccess()) {
        return;
    }

    LockedBlackboardRO bb(*getBlackboard());
    try {
        if (bb.get<int64_t>("mappedIntValue") == -9 && bb.get<uint64_t>("mappedUintValue") == 5 && bb.get<bool>("mappedBoolValue") == true &&
                bb.get<std::string>("mappedStringValue") == "test" && bb.get<double>("mappedDoubleValue") == 4.5) {
            setSuccess();
        } else {
            std::stringstream ss;
            ss << "ValueMappingTestBeh: One or more of the mapped values are wrong:\n";
            ss << "key: mappedIntValue, value: " << bb.get<int64_t>("mappedIntValue") << ", expected: -9\n";
            ss << "key: mappedUintValue, value: " << bb.get<uint64_t>("mappedUintValue") << ", expected: 5\n";
            ss << "key: mappedBoolValue, value: " << bb.get<bool>("mappedBoolValue") << ", expected: true\n";
            ss << "key: mappedStringValue, value: " << bb.get<std::string>("mappedStringValue") << ", expected: test\n";
            ss << "key: mappedDoubleValue, value: " << bb.get<double>("mappedDoubleValue") << ", expected: 4.5\n";

            LockedBlackboardRW gb(*getGlobalBlackboard());
            gb.set("ValueMappingTestBehError", ss.str());
        }
    } catch (const BlackboardException& e) {
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("ValueMappingTestBehError", std::string(e.what()));
    }
}

std::unique_ptr<ValueMappingTestBeh> ValueMappingTestBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<ValueMappingTestBeh>(context);
}

} /* namespace alica */
