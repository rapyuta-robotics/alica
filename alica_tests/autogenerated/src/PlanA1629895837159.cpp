#include "PlanA1629895837159.h"
/*PROTECTED REGION ID(eph1629895837159) ENABLED START*/
// Add additional options here
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
    /*PROTECTED REGION END*/
}
PlanA1629895837159::~PlanA1629895837159()
{
    /*PROTECTED REGION ID(dcon1629895837159) ENABLED START*/
    // Add additional options here
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
/*PROTECTED REGION END*/
} // namespace alica
