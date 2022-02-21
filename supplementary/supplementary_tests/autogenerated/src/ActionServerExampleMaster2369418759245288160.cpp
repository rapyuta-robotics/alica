#include "ActionServerExampleMaster2369418759245288160.h"
/*PROTECTED REGION ID(eph2369418759245288160) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ActionServerExampleMaster (2369418759245288160)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 3264799698587952617)
//
// States:
//   - RunActionServer (1749352633184971627)
ActionServerExampleMaster2369418759245288160::ActionServerExampleMaster2369418759245288160(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con2369418759245288160) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ActionServerExampleMaster2369418759245288160::~ActionServerExampleMaster2369418759245288160()
{
    /*PROTECTED REGION ID(dcon2369418759245288160) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 3264799698587952617
 */
std::shared_ptr<UtilityFunction> UtilityFunction2369418759245288160::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(2369418759245288160) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods2369418759245288160) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
