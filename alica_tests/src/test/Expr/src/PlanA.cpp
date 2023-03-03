#include <alica_tests/test_sched_world_model.h>
#include <libalica-tests/plans/PlanA.h>

namespace alica
{
PlanA::PlanA(PlanContext& context)
        : AlicaTestsPlan(context)
{
    _inRunContext = false;
}
void PlanA::onInit()
{
    _wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    _wm->execOrder += "PlanA::Init\n";
    _inRunContext = true;
}

void PlanA::run()
{
    _wm->planARunCalled = true;
    if (!_inRunContext) {
        _wm->planARunOutOfOrder = true;
    }
}

void PlanA::onTerminate()
{
    _inRunContext = false;
    _wm->execOrder += "PlanA::Term\n";
}

} // namespace alica
