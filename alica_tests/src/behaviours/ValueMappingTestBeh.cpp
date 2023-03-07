#include <alica_tests/behaviours/ValueMappingTestBeh.h>
#include <memory>

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
    if (bb.get<int>("inputInt") == -9 && bb.get<uint64_t>("inputUint") == 5 && bb.get<bool>("inputBool") == true &&
            bb.get<std::string>("inputString") == "test" && bb.get<double>("inputDouble") == 4.5) {
        setSuccess();
    }
}

std::unique_ptr<ValueMappingTestBeh> ValueMappingTestBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<ValueMappingTestBeh>(context);
}

} /* namespace alica */
