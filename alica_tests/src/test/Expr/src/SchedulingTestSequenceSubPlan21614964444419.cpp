#include "SchedulingTestSequenceSubPlan21614964444419.h"
/*PROTECTED REGION ID(eph1614964444419) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SchedulingTestSequenceSubPlan2 (1614964444419)
//
// Tasks:
//   - SchedulerTestSubPlanEntrypoint (1613372009777) (Entrypoint: 1614964892485)
//
// States:
//   - InitSequenceSubPlan1 (1614964894440)
SchedulingTestSequenceSubPlan21614964444419::SchedulingTestSequenceSubPlan21614964444419(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1614964444419) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SchedulingTestSequenceSubPlan21614964444419::~SchedulingTestSequenceSubPlan21614964444419()
{
    /*PROTECTED REGION ID(dcon1614964444419) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: SchedulerTestSubPlanEntrypoint  -> EntryPoint-ID: 1614964892485
 */
std::shared_ptr<UtilityFunction> UtilityFunction1614964444419::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1614964444419) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1614964444419) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
