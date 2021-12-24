#include "SerializationSubPlanC2359124678252958039.h"
/*PROTECTED REGION ID(eph2359124678252958039) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  SerializationSubPlanC (2359124678252958039)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 2572645828071056843)
//
// States:
//   - EntryState (2604890859274745239)
SerializationSubPlanC2359124678252958039::SerializationSubPlanC2359124678252958039(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con2359124678252958039) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
SerializationSubPlanC2359124678252958039::~SerializationSubPlanC2359124678252958039()
{
    /*PROTECTED REGION ID(dcon2359124678252958039) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 2572645828071056843
 */
std::shared_ptr<UtilityFunction> UtilityFunction2359124678252958039::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2359124678252958039) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2359124678252958039) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
