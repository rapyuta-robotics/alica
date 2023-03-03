#include "alica_tests/TestWorldModel.h"
#include <libalica-tests/behaviours/TestInheritBlackboardBehaviour.h>

#include <memory>

namespace alica
{

TestInheritBlackboardBehaviour::TestInheritBlackboardBehaviour(BehaviourContext& context)
        : DomainBehaviour(context)
{
}
TestInheritBlackboardBehaviour::~TestInheritBlackboardBehaviour() {}
void TestInheritBlackboardBehaviour::run() {}
void TestInheritBlackboardBehaviour::initialiseParameters()
{
    auto wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alicaTests::TestWorldModel>>("worldmodel");
    LockedBlackboardRW bb(*(getBlackboard()));
    bb.set<int64_t>("masterKey", 3);
    wm->passedParameters["masterKeyInBehavior"] = bb.get<int64_t>("masterKey");
    if (bb.hasValue("behaviorKey")) {
        wm->passedParameters["hasBehaviorKey"] = 3;
    } else {
        wm->passedParameters["hasBehaviorKey"] = 4;
    }
}
std::unique_ptr<TestInheritBlackboardBehaviour> TestInheritBlackboardBehaviour::create(alica::BehaviourContext& context)
{
    return std::make_unique<TestInheritBlackboardBehaviour>(context);
}

} /* namespace alica */
