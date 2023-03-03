#include <alica_tests/test_sched_world_model.h>
#include <libalica-tests/plans/PlanBA.h>

namespace alica
{
PlanBA::PlanBA(PlanContext& context)
        : AlicaTestsPlan(context)
{
}
void PlanBA::onInit()
{
    _wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    _wm->execOrder += "PlanBA::Init\n";
}

void PlanBA::onTerminate()
{
    _wm->execOrder += "PlanBA::Term\n";
}
} // namespace alica
