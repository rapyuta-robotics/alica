#include "DynamicTaskLC2140075868731779222.h"
/*PROTECTED REGION ID(eph2140075868731779222) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  DynamicTaskLC (2140075868731779222)
//
// Tasks:
//   - DynamicTask (1163169622598227531) (Entrypoint: 3626583666892196532)
//
// States:
//   - LD (3534468625273851172)
DynamicTaskLC2140075868731779222::DynamicTaskLC2140075868731779222(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con2140075868731779222) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
DynamicTaskLC2140075868731779222::~DynamicTaskLC2140075868731779222()
{
    /*PROTECTED REGION ID(dcon2140075868731779222) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DynamicTask  -> EntryPoint-ID: 3626583666892196532
 */
std::shared_ptr<UtilityFunction> UtilityFunction2140075868731779222::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2140075868731779222) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2140075868731779222) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
