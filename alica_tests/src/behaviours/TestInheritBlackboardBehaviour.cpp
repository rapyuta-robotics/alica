#include <alica_tests/behaviours/TestInheritBlackboardBehaviour.h>

#include <cstdint>
#include <memory>

namespace alica
{

TestInheritBlackboardBehaviour::TestInheritBlackboardBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
TestInheritBlackboardBehaviour::~TestInheritBlackboardBehaviour() {}
void TestInheritBlackboardBehaviour::run() {}
void TestInheritBlackboardBehaviour::initialiseParameters()
{
    int64_t masterKeyInBehavior = 0;
    {
        LockedBlackboardRW bb(*getBlackboard());
        masterKeyInBehavior = bb.get<int64_t>("masterKey");
        bb.set<int64_t>("behaviourKey", 323);
    }
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set("masterKeyInBehaviour", masterKeyInBehavior);
}
std::unique_ptr<TestInheritBlackboardBehaviour> TestInheritBlackboardBehaviour::create(alica::BehaviourContext& context)
{
    return std::make_unique<TestInheritBlackboardBehaviour>(context);
}

} /* namespace alica */
