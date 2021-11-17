#include "VariableHandling/Lvl31524452836022.h"
/*PROTECTED REGION ID(eph1524452836022) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  Lvl3 (1524452836022)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1524452836024)
//
// States:
//   - NewState (1524452836023)
Lvl31524452836022::Lvl31524452836022()
        : DomainPlan()
{
    /*PROTECTED REGION ID(con1524452836022) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
Lvl31524452836022::~Lvl31524452836022()
{
    /*PROTECTED REGION ID(dcon1524452836022) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): Lvl3 Runtime Condition, (Comment) :

/**
 * Available Vars:
 *	- L3A (1524453054226)
 *	- L3B (1524453060294)
 */
bool RunTimeCondition1524452937477::evaluate(std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(1524452937477) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1524452836024
 */
std::shared_ptr<UtilityFunction> UtilityFunction1524452836022::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1524452836022) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1524452836022) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
