#include <alica_tests/PlanAA.h>
#include <alica_tests/test_sched_world_model.h>

namespace alica
{
PlanAA::PlanAA(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void PlanAA::onInit()
{
    _wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    _wm->execOrder += "PlanAA::Init\n";
}

void PlanAA::onTerminate()
{
    _wm->execOrder += "PlanAA::Term\n";
}
} // namespace alica
