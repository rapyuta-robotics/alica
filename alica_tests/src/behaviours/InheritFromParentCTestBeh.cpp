#include <alica_tests/behaviours/InheritFromParentCTestBeh.h>
#include <memory>

namespace alica
{

InheritFromParentCTestBeh::InheritFromParentCTestBeh(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
void InheritFromParentCTestBeh::initialiseParameters()
{
    LockedBlackboardRW bb(*getBlackboard());
    if (bb.hasValue("boolValueA") || bb.hasValue("stringValueA") || bb.hasValue("intValueA") || bb.hasValue("uintValueA") || bb.hasValue("doubleValueA")) {
        LockedBlackboardRW gb(*getGlobalBlackboard());
        gb.set("testError", "Child NotInheritFromParentBTestPlan inherits blackboard from parent InheritFromParentATestPlan, but should not inherit!");
        return;
    }

    if (bb.get<bool>("boolValueB") == false && bb.get<double>("doubleValueB") == 3.0 && bb.get<int64_t>("intValueB") == 3 &&
                    bb.get<uint64_t>("uintValueB") == 3u,
            bb.get<std::string>("stringValueB") == "stringFromB") {
        setSuccess();
    }
}

std::unique_ptr<InheritFromParentCTestBeh> InheritFromParentCTestBeh::create(alica::BehaviourContext& context)
{
    return std::make_unique<InheritFromParentCTestBeh>(context);
}

} /* namespace alica */
