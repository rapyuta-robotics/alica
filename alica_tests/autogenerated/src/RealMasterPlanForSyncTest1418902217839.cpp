#include "RealMasterPlanForSyncTest1418902217839.h"
/*PROTECTED REGION ID(eph1418902217839) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  RealMasterPlanForSyncTest (1418902217839)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1418902217841)
//
// States:
//   - NewState (1418902217840)
RealMasterPlanForSyncTest1418902217839::RealMasterPlanForSyncTest1418902217839()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1418902217839) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
RealMasterPlanForSyncTest1418902217839::~RealMasterPlanForSyncTest1418902217839()
{
    /*PROTECTED REGION ID(dcon1418902217839) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1418902217841
 */
std::shared_ptr<UtilityFunction> UtilityFunction1418902217839::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1418902217839) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1418902217839) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
