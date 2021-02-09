#include "BehaviourTriggerTestPlan1428508768572.h"
/*PROTECTED REGION ID(eph1428508768572) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:BehaviourTriggerTestPlan1428508768572
BehaviourTriggerTestPlan1428508768572::BehaviourTriggerTestPlan1428508768572()
        : DomainPlan("BehaviourTriggerTestPlan1428508768572")
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
std::shared_ptr<UtilityFunction> UtilityFunction1428508768572::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1428508768572) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment:
 *
 * Abstract plans in current state:
 *   - TriggerA (1428508297492)
 *   - TriggerB (1428508316905)
 *   - TriggerC (1428508355209)
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1428508768574)
 *
 * States in plan:
 *   - NewState (1428508768573)
 *   - NewState (1429017227839)
 *
 * Variables of precondition:
 */
bool PreCondition1429017236633::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1429017235181) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}
} // namespace alica
