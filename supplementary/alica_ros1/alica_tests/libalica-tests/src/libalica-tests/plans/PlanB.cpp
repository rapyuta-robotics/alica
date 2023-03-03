#include <alica_tests/test_sched_world_model.h>
#include <libalica-tests/plans/PlanB.h>

namespace alica
{
PlanB::PlanB(PlanContext& context)
        : AlicaTestsPlan(context)
{
}

void PlanB::onInit()
{
    _wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    _wm->execOrder += "PlanB::Init\n";
}

void PlanB::onTerminate()
{
    _wm->execOrder += "PlanB::Term\n";
}
} // namespace alica
