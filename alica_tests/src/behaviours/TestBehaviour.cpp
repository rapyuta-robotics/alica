#include <alica_tests/behaviours/TestBehaviour.h>
#include <memory>

namespace alica
{
TestBehaviour::TestBehaviour(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
TestBehaviour::~TestBehaviour() {}
void TestBehaviour::run()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());

    if (gb.get<bool>("executeBehaviourRunCalled")) {
        return;
    }
    gb.set("execOrder", gb.get<std::string>("execOrder") + "TestBehaviour::Run\n");
    gb.set("executeBehaviourRunCalled", true);
}
void TestBehaviour::initialiseParameters()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set("execOrder", gb.hasValue("execOrder") ? gb.get<std::string>("execOrder") + "TestBehaviour::Init\n" : "TestBehaviour::Init\n");
    gb.set("executeBehaviourRunCalled", false);
}

void TestBehaviour::onTermination()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set("execOrder", gb.get<std::string>("execOrder") + "TestBehaviour::Term\n");
}

std::unique_ptr<TestBehaviour> TestBehaviour::create(alica::BehaviourContext& context)
{
    return std::make_unique<TestBehaviour>(context);
}
} /* namespace alica */
