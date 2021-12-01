#include "PlanB1629895853508.h"
/*PROTECTED REGION ID(eph1629895853508) ENABLED START*/
// Add additional options here
#include "engine/PlanInterface.h"
#include <alica_tests/test_sched_world_model.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PlanB (1629895853508)
//
// Tasks:
//   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1629896055805)
//
// States:
//   - PlanBA (1629896057548)
PlanB1629895853508::PlanB1629895853508(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1629895853508) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanB1629895853508::~PlanB1629895853508()
{
    /*PROTECTED REGION ID(dcon1629895853508) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestEntrypoint  -> EntryPoint-ID: 1629896055805
 */
std::shared_ptr<UtilityFunction> UtilityFunction1629895853508::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1629895853508) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1629895853508) ENABLED START*/
// Add additional options here
void PlanB1629895853508::onInit()
{
    _wm = dynamic_cast<alica_test::SchedWM*>(getWorldModel());
    _wm->execOrder += "PlanB::Init\n";
}

void PlanB1629895853508::onTerminate()
{
    _wm->execOrder += "PlanB::Term\n";
}
/*PROTECTED REGION END*/
} // namespace alica
