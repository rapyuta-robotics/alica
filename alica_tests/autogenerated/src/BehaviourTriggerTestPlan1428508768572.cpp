#include "BehaviourTriggerTestPlan1428508768572.h"
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
//   - NewState (1429017227839)
BehaviourTriggerTestPlan1428508768572::BehaviourTriggerTestPlan1428508768572()
        : DomainPlan()
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

void BehaviourTriggerTestPlan1428508768572::run(void* msg)
{
    /*PROTECTED REGION ID(runBehaviourTriggerTestPlan1428508768572) ENABLED START*/
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
 * Transition: MISSING_NAME (1429017235181)
 *   - Comment:
 *   - Source2Dest: NewState --> NewState
 *
 * Precondition: MISSING_NAME (1429017236633)
 *   - Enabled: true
 *   - PluginName: DefaultPlugin
 *   - ConditionString:
 *   - Variables:
 *   - Quantifiers:
 *
 * Abstract Plans in NewState:
 *   - TriggerA (1428508297492)
 *   - TriggerB (1428508316905)
 *   - TriggerC (1428508355209)
 */
bool PreCondition1429017236633::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1429017235181) ENABLED START*/
    return false;
    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1428508768572) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
