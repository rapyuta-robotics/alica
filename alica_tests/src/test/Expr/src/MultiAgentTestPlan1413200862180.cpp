#include <alica_tests/MultiAgentTestPlan1413200862180.h>
/*PROTECTED REGION ID(eph1413200862180) ENABLED START*/
// Add additional using directives here
#include <alica_tests/TestWorldModel.h>
#include <engine/AlicaEngine.h>
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  MultiAgentTestPlan (1413200862180)
//
// Tasks:
//   - AttackTask (1407153522080) (Entrypoint: 1413200877337)//   - DefaultTask (1225112227903) (Entrypoint: 1413200890537)//   - DefaultTask (1225112227903)
//   (Entrypoint: 1413807260446)
//
// States:
//   - OtherState (1413200877336)
//   - State1 (1413200910490)
//   - State2 (1413201030936)
//   - NewSuccessState1 (1413201164999)
//   - NewSuccessState2 (1413552736921)
//   - Idle (1413807264574)
MultiAgentTestPlan1413200862180::MultiAgentTestPlan1413200862180(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1413200862180) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
MultiAgentTestPlan1413200862180::~MultiAgentTestPlan1413200862180()
{
    /*PROTECTED REGION ID(dcon1413200862180) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: AttackTask  -> EntryPoint-ID: 1413200877337
 * Task: DefaultTask  -> EntryPoint-ID: 1413200890537
 * Task: DefaultTask  -> EntryPoint-ID: 1413807260446
 */

UtilityFunction1413200862180::UtilityFunction1413200862180(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1413200862180::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1413200862180) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan, _logger);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1413200862180) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
