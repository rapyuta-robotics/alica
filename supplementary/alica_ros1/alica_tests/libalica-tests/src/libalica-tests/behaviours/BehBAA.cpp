#include <libalica-tests/behaviours/BehBAA.h>
#include <memory>

#include <alica_tests/test_sched_world_model.h>

namespace alica
{
int BehBAA::runCount;

BehBAA::BehBAA(BehaviourContext& context)
        : DomainBehaviour(context)
{
}
BehBAA::~BehBAA() {}
void BehBAA::run()
{
    ++runCount;
}
void BehBAA::initialiseParameters()
{
    _wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    _wm->execOrder += "BehBAA::Init\n";
    runCount = 0;
}
void BehBAA::onTermination()
{
    runCount = 0;
    _wm->execOrder += "BehBAA::Term\n";
}
std::unique_ptr<BehBAA> BehBAA::create(alica::BehaviourContext& context)
{
    return std::make_unique<BehBAA>(context);
}

} /* namespace alica */
