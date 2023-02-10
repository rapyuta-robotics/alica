#include <alica_tests/Behaviour/TestParameterPassingBehaviour.h>
#include <memory>

#include "alica_tests/TestWorldModel.h"

namespace alica
{

TestParameterPassingBehaviour::TestParameterPassingBehaviour(BehaviourContext& context)
        : DomainBehaviour(context)
{
}
TestParameterPassingBehaviour::~TestParameterPassingBehaviour() {}
void TestParameterPassingBehaviour::run()
{
    setSuccess();
}
void TestParameterPassingBehaviour::initialiseParameters()
{
    auto wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<int64_t>("behaviorKey", 2);
    wm->passedParameters["behaviorKey"] = bb.get<int64_t>("behaviorKey");

    auto value = bb.get<int64_t>("behaviorInputKey");
    if (value == 5) {
        wm->passedParameters["behaviorInputKey"] = value;
    } else if (value == 7) {
        wm->passedParameters["behaviorSecondInputKey"] = value;
    }

    bb.set<int64_t>("behaviorOutputKey", 6);
}

std::unique_ptr<TestParameterPassingBehaviour> TestParameterPassingBehaviour::create(alica::BehaviourContext& context)
{
    return std::make_unique<TestParameterPassingBehaviour>(context);
}
} /* namespace alica */
