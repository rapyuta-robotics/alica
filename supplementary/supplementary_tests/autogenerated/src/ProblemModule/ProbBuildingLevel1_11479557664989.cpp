#include "ProblemModule/ProbBuildingLevel1_11479557664989.h"
/*PROTECTED REGION ID(eph1479557664989) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ProbBuildingLevel1_1 (1479557664989)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1479557690963)
//
// States:
//   - MiddleState (1479557690962)
ProbBuildingLevel1_11479557664989::ProbBuildingLevel1_11479557664989(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1479557664989) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ProbBuildingLevel1_11479557664989::~ProbBuildingLevel1_11479557664989()
{
    /*PROTECTED REGION ID(dcon1479557664989) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1479557690963
 */
std::shared_ptr<UtilityFunction> UtilityFunction1479557664989::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1479557664989) ENABLED START*/
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1479557664989) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
