#include <alica_tests/behaviours/BehAAA.h>
#include <memory>

#include <alica_tests/test_sched_world_model.h>

namespace alica
{
int BehAAA::runCount;

BehAAA::BehAAA(BehaviourContext& context)
        : BasicBehaviour(context)
{
}
BehAAA::~BehAAA() {}
void BehAAA::run()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set(getName() + "RunCount", gb.get<int64_t>(getName() + "RunCount") + 1);
    if (!_inRunContext) {
        gb.set(getName() + "RunOutOfOrder", true);
    }
}
void BehAAA::initialiseParameters()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set(getName() + "RunCount", 0);
    gb.set(getName() + "RunCalled", true);
    gb.set("execOrder", gb.hasValue("execOrder") ? gb.get<std::string>("execOrder") + getName() + "::Init\n" : getName() + "::Init\n");

    _inRunContext = true;
}

void BehAAA::onTermination()
{
    runCount = 0;
    _inRunContext = false;
    LockedBlackboardRW gb(*getGlobalBlackboard());
    gb.set(getName() + "RunCount", 0);
    gb.set("execOrder", gb.get<std::string>("execOrder") + getName() + "::Term\n");
}

std::unique_ptr<BehAAA> BehAAA::create(alica::BehaviourContext& context)
{
    return std::make_unique<BehAAA>(context);
}

} /* namespace alica */
