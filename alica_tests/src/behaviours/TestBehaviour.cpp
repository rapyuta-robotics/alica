#include <alica_tests/behaviours/TestBehaviour.h>
#include <memory>

#include <alica_tests/test_sched_world_model.h>
namespace alica
{
TestBehaviour::TestBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
TestBehaviour::~TestBehaviour() {}
void TestBehaviour::run()
{
    auto wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");

    if (wm->executeBehaviourRunCalled) {
        return;
    }
    wm->execOrder += "TestBehaviour::Run\n";
    wm->executeBehaviourRunCalled = true;
}
void TestBehaviour::initialiseParameters()
{
    auto wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    wm->execOrder += "TestBehaviour::Init\n";
    wm->executeBehaviourRunCalled = false;
}

void TestBehaviour::onTermination()
{
    auto wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    wm->execOrder += "TestBehaviour::Term\n";
}

std::unique_ptr<TestBehaviour> TestBehaviour::create(alica::BehaviourContext& context)
{
    return std::make_unique<TestBehaviour>(context);
}
} /* namespace alica */
