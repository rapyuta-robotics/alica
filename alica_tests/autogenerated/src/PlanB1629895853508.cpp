#include "PlanB1629895853508.h"
/*PROTECTED REGION ID(eph1629895853508) ENABLED START*/
// Add additional options here
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
PlanB1629895853508::PlanB1629895853508()
        : DomainPlan()
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

void PlanB1629895853508::run(void* msg)
{
    /*PROTECTED REGION ID(runPlanB1629895853508) ENABLED START*/
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
    alica_test::SchedWM::instance().execOrder += "PlanB::Init\n";
}

void PlanB1629895853508::onTerminate()
{
    alica_test::SchedWM::instance().execOrder += "PlanB::Term\n";
}
/*PROTECTED REGION END*/
} // namespace alica
