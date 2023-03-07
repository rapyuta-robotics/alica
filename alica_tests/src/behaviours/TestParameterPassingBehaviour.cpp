#include <alica_tests/behaviours/TestParameterPassingBehaviour.h>
#include <memory>

#include "alica_tests/TestWorldModel.h"

namespace alica
{

TestParameterPassingBehaviour::TestParameterPassingBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
TestParameterPassingBehaviour::~TestParameterPassingBehaviour() {}
void TestParameterPassingBehaviour::run()
{
    if (isSuccess()) {
        return;
    }

    LockedBlackboardRO gb(*getGlobalBlackboard());
    if (gb.get<int64_t>("param1") == 5 && gb.get<int64_t>("param2") == 7) {
        setSuccess();
    }
}
void TestParameterPassingBehaviour::initialiseParameters()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    LockedBlackboardRW bb(*getBlackboard());
    bb.set<int64_t>("behaviorKey", 2);

    auto value = bb.get<int64_t>("behaviorInputKey");
    if (value == 5) {
        gb.set("param1", value);
    } else if (value == 7) {
        gb.set("param2", value);
    }

    bb.set<int64_t>("behaviorOutputKey", 6);
}

std::unique_ptr<TestParameterPassingBehaviour> TestParameterPassingBehaviour::create(alica::BehaviourContext& context)
{
    return std::make_unique<TestParameterPassingBehaviour>(context);
}
} /* namespace alica */
