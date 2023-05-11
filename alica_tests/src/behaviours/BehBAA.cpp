#include <alica_tests/behaviours/BehBAA.h>
#include <memory>

#include <alica_tests/test_sched_world_model.h>

namespace alica
{
int BehBAA::runCount;

BehBAA::BehBAA(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
BehBAA::~BehBAA() {}
void BehBAA::run()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set(getName() + "RunCount", gb.get<int64_t>(getName() + "RunCount") + 1);
}
void BehBAA::initialiseParameters()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set(getName() + "RunCount", 0);
    gb.set("execOrder", gb.get<std::string>("execOrder") + getName() + "::Init\n");
}
void BehBAA::onTermination()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set(getName() + "RunCount", 0);
    gb.set("execOrder", gb.get<std::string>("execOrder") + getName() + "::Term\n");
}
std::unique_ptr<BehBAA> BehBAA::create(alica::BehaviourContext& context)
{
    return std::make_unique<BehBAA>(context);
}

} /* namespace alica */
