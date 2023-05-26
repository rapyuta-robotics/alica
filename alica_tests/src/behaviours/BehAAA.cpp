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
    if (isSuccess()) {
        return;
    }
    LockedBlackboardRW gb(*getGlobalBlackboard());
    std::vector<std::string>& execOrder = gb.get<std::vector<std::string>>("execOrder");
    execOrder.emplace_back(getName() + "::Run");
    setSuccess();
}
void BehAAA::initialiseParameters()
{
    LockedBlackboardRW gb(*getGlobalBlackboard());
    std::vector<std::string>& execOrder = gb.get<std::vector<std::string>>("execOrder");
    execOrder.emplace_back(getName() + "::Init");

    _inRunContext = true;
}

void BehAAA::onTermination()
{
    _inRunContext = false;
    LockedBlackboardRW gb(*getGlobalBlackboard());
    std::vector<std::string>& execOrder = gb.get<std::vector<std::string>>("execOrder");
    execOrder.emplace_back(getName() + "::Term");
}

std::unique_ptr<BehAAA> BehAAA::create(alica::BehaviourContext& context)
{
    return std::make_unique<BehAAA>(context);
}

} /* namespace alica */
