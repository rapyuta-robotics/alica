#include <alica_tests/MultiAgentTestMaster1413200842973.h>
/*PROTECTED REGION ID(eph1413200842973) ENABLED START*/
// Add additional using directives here
#include <alica_tests/TestWorldModel.h>
#include <engine/AlicaEngine.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MultiAgentTestMaster (1413200842973)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1413200842975)
//
// States:
//   - Init (1413200842974)
//   - Start (1413201213955)
//   - Finished (1413201380359)
MultiAgentTestMaster1413200842973::MultiAgentTestMaster1413200842973(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1413200842973) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MultiAgentTestMaster1413200842973::~MultiAgentTestMaster1413200842973()
{
    /*PROTECTED REGION ID(dcon1413200842973) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1413200842975
 */

UtilityFunction1413200842973::UtilityFunction1413200842973(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1413200842973::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1413200842973) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan, _logger);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1413200842973) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
