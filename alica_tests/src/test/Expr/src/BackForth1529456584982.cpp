#include <alica_tests/BackForth1529456584982.h>
/*PROTECTED REGION ID(eph1529456584982) ENABLED START*/
// Add additional using directives here
#include <alica_tests/SimpleSwitches.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  BackForth (1529456584982)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1529456584984)
//
// States:
//   - First (1529456584983)
//   - Second (1529456591410)
BackForth1529456584982::BackForth1529456584982(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1529456584982) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
BackForth1529456584982::~BackForth1529456584982()
{
    /*PROTECTED REGION ID(dcon1529456584982) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1529456584984
 */

UtilityFunction1529456584982::UtilityFunction1529456584982(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1529456584982::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1529456584982) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan, _logger);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1529456584982) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
