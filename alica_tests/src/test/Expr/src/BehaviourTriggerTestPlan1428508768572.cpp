#include <alica_tests/BehaviourTriggerTestPlan1428508768572.h>
/*PROTECTED REGION ID(eph1428508768572) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  BehaviourTriggerTestPlan (1428508768572)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1428508768574)
//
// States:
//   - NewState (1428508768573)
BehaviourTriggerTestPlan1428508768572::BehaviourTriggerTestPlan1428508768572(PlanContext& context)
        : DomainPlan(context)
{
    /*PROTECTED REGION ID(con1428508768572) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
BehaviourTriggerTestPlan1428508768572::~BehaviourTriggerTestPlan1428508768572()
{
    /*PROTECTED REGION ID(dcon1428508768572) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

/**
 * Task: DefaultTask  -> EntryPoint-ID: 1428508768574
 */

UtilityFunction1428508768572::UtilityFunction1428508768572(IAlicaLogger& logger)
        : BasicUtilityFunction(logger)
{
}

std::shared_ptr<UtilityFunction> UtilityFunction1428508768572::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1428508768572) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan, _logger);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1428508768572) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
