#include "PlanA1629895837159.h"
/*PROTECTED REGION ID(eph1629895837159) ENABLED START*/
// Add additional options here
#include <alica_tests/test_sched_world_model.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PlanA (1629895837159)
//
// Tasks:
//   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1629895952886)
//
// States:
//   - PlanAA (1629895956631)
PlanA1629895837159::PlanA1629895837159()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1629895837159) ENABLED START*/
    // Add additional options here
    _inRunContext = false;
    /*PROTECTED REGION END*/
}
PlanA1629895837159::~PlanA1629895837159()
{
    /*PROTECTED REGION ID(dcon1629895837159) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

void PlanA1629895837159::run(void* msg)
{
    /*PROTECTED REGION ID(runPlanA1629895837159) ENABLED START*/
    // Add additional options here
    auto& wm = alica_test::SchedWM::instance();
    wm.planARunCalled = true;
    if (!_inRunContext) {
        wm.planARunOutOfOrder = true;
    }
    /*PROTECTED REGION END*/
}
/**
 * Task: SchedulerTestEntrypoint  -> EntryPoint-ID: 1629895952886
 */
std::shared_ptr<UtilityFunction> UtilityFunction1629895837159::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1629895837159) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1629895837159) ENABLED START*/
// Add additional options here
void PlanA1629895837159::onInit()
{
    alica_test::SchedWM::instance().execOrder += "PlanA::Init\n";
    _inRunContext = true;
}

void PlanA1629895837159::onTerminate()
{
    _inRunContext = false;
    alica_test::SchedWM::instance().execOrder += "PlanA::Term\n";
}

/*PROTECTED REGION END*/
} // namespace alica
