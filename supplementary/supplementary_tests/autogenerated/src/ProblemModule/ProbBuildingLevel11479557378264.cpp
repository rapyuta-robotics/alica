#include "ProblemModule/ProbBuildingLevel11479557378264.h"
/*PROTECTED REGION ID(eph1479557378264) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ProbBuildingLevel1 (1479557378264)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1479557378266)
//
// States:
//   - PTState (1479557378265)
ProbBuildingLevel11479557378264::ProbBuildingLevel11479557378264()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1479557378264) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ProbBuildingLevel11479557378264::~ProbBuildingLevel11479557378264()
{
    /*PROTECTED REGION ID(dcon1479557378264) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1479557378266
 */
std::shared_ptr<UtilityFunction> UtilityFunction1479557378264::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1479557378264) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1479557378264) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
