#include "VariableHandling/Lvl11524452759599.h"
/*PROTECTED REGION ID(eph1524452759599) ENABLED START*/
// Add additional using directives here
bool vhStartCondition = false;
/*PROTECTED REGION END*/

namespace alica
{
// Plan:Lvl1
// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): Lvl1 Runtime Condition, (Comment) :

/**
 * Available Vars:
 *	- L1A (1524453326397)
 *	- L1B (1524453331530)
 */
bool RunTimeCondition1524453470580::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1524453470580) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1524452759601
 */
std::shared_ptr<UtilityFunction> UtilityFunction1524452759599::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1524452759599) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}
/**
 * Outgoing transition:
 *   - Name: MISSING_NAME, ConditionString: , Comment: Lvl1 Transition
 *
 * Abstract plans in current state:
 *
 * Tasks in plan:
 *   - DefaultTask (1225112227903) (Entrypoint: 1524452759601)
 *
 * States in plan:
 *   - NewState (1524452759600)
 *   - BeforeTrans (1524453481856)
 *
 * Variables of preconditon:
 */
bool PreCondition1524453491764::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1524453490345) ENABLED START*/
    return vhStartCondition;
    /*PROTECTED REGION END*/
}
} // namespace alica
