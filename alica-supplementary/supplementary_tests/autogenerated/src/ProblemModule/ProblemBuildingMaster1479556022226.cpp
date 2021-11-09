#include "ProblemModule/ProblemBuildingMaster1479556022226.h"
/*PROTECTED REGION ID(eph1479556022226) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ProblemBuildingMaster (1479556022226)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1479556022228)
//
// States:
//   - State1 (1479556022227)
//   - State2 (1479557585252)
ProblemBuildingMaster1479556022226::ProblemBuildingMaster1479556022226()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1479556022226) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ProblemBuildingMaster1479556022226::~ProblemBuildingMaster1479556022226()
{
    /*PROTECTED REGION ID(dcon1479556022226) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1479556022228
 */
std::shared_ptr<UtilityFunction> UtilityFunction1479556022226::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1479556022226) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Transition: MISSING_NAME (1479557591331)
 *   - Comment:
 *   - Source2Dest: State1 --> State2
 *
 * Precondition: MISSING_NAME (1479557592662)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *	   - PBMX (1479557337956)
 *	   - PBMY (1479557345903)
 *   - Quantifiers:
 *	   - MISSING_NAME (1479557619214)
 *
 * Abstract Plans in State1:
 *   - ProbBuildingLevel1 (1479557378264)
 */
bool PreCondition1479557592662::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1479557591331) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1479556022226) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
