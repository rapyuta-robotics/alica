#include <supplementary_tests/ProblemModule/ProblemBuildingMaster1479556022226.h>
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
ProblemBuildingMaster1479556022226::ProblemBuildingMaster1479556022226(PlanContext& context)
        : DomainPlan(context)
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

UtilityFunction1479556022226::UtilityFunction1479556022226(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1479556022226::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1479556022226) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1479556022226) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
