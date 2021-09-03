#include "PlanAA1629895864090.h"
/*PROTECTED REGION ID(eph1629895864090) ENABLED START*/
// Add additional options here
#include <alica_tests/test_sched_world_model.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  PlanAA (1629895864090)
//
// Tasks:
//   - SchedulerTestEntrypoint (1613371619454) (Entrypoint: 1629896015785)
//
// States:
//   - BehAAA (1629896006533)
PlanAA1629895864090::PlanAA1629895864090()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1629895864090) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
PlanAA1629895864090::~PlanAA1629895864090()
{
    /*PROTECTED REGION ID(dcon1629895864090) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

void PlanAA1629895864090::run(void* msg)
{
    /*PROTECTED REGION ID(runPlanAA1629895864090) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
/**
 * Task: SchedulerTestEntrypoint  -> EntryPoint-ID: 1629896015785
 */
std::shared_ptr<UtilityFunction> UtilityFunction1629895864090::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1629895864090) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1629895864090) ENABLED START*/
// Add additional options here
void PlanAA1629895864090::onInit()
{
    alica_test::SchedWM::instance().execOrder += "PlanAA::Init\n";
}

void PlanAA1629895864090::onTerminate()
{
    alica_test::SchedWM::instance().execOrder += "PlanAA::Term\n";
}
/*PROTECTED REGION END*/
} // namespace alica
