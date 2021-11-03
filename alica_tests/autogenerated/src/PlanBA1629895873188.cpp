#include "PlanBA1629895873188.h"
/*PROTECTED REGION ID(eph1629895873188) ENABLED START*/
// Add additional options here
#include <alica_tests/test_sched_world_model.h>
#include "engine/PlanInterface.h"
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PlanBA (1629895873188)
//
// Tasks:
//   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1629896091322)
//
// States:
//   - BehBAA (1629896094706)
PlanBA1629895873188::PlanBA1629895873188()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1629895873188) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanBA1629895873188::~PlanBA1629895873188()
{
    /*PROTECTED REGION ID(dcon1629895873188) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestEntrypoint  -> EntryPoint-ID: 1629896091322
 */
std::shared_ptr<UtilityFunction> UtilityFunction1629895873188::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1629895873188) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1629895873188) ENABLED START*/
// Add additional options here
void PlanBA1629895873188::onInit()
{
    alica_test::SchedWM* wm = dynamic_cast<alica_test::SchedWM*>(getPlanContext().getRunningPlan()->getWorldModel());
    wm->execOrder += "PlanBA::Init\n";
}

void PlanBA1629895873188::onTerminate()
{
    alica_test::SchedWM* wm = dynamic_cast<alica_test::SchedWM*>(getPlanContext().getRunningPlan()->getWorldModel());
    wm->execOrder += "PlanBA::Term\n";
}
/*PROTECTED REGION END*/
} // namespace alica
