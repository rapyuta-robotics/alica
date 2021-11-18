#include "ConstraintTestPlan1414068524245.h"
/*PROTECTED REGION ID(eph1414068524245) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:  ConstraintTestPlan (1414068524245)
//
// Tasks:
//   - DefaultTask (1225112227903) (Entrypoint: 1414068524247)
//
// States:
//   - constraintRunner (1414068524246)
ConstraintTestPlan1414068524245::ConstraintTestPlan1414068524245(IAlicaWorldModel* wm)
        : DomainPlan(wm)
{
    /*PROTECTED REGION ID(con1414068524245) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}
ConstraintTestPlan1414068524245::~ConstraintTestPlan1414068524245()
{
    /*PROTECTED REGION ID(dcon1414068524245) ENABLED START*/
    // Add additional options here
    /*PROTECTED REGION END*/
}

// Check of RuntimeCondition - (Name): NewRuntimeCondition, (ConditionString): , (Comment) :

/**
 * Available Vars:
 *	- X (1414068572540)
 *	- Y (1414068576620)
 */
bool RunTimeCondition1414068566297::evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel& wm)
{
    /*PROTECTED REGION ID(1414068566297) ENABLED START*/
    return true;
    /*PROTECTED REGION END*/
}
/**
 * Task: DefaultTask  -> EntryPoint-ID: 1414068524247
 */
std::shared_ptr<UtilityFunction> UtilityFunction1414068524245::getUtilityFunction(Plan* plan)
{
    /*PROTECTED REGION ID(1414068524245) ENABLED START*/

    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;

    /*PROTECTED REGION END*/
}

/*PROTECTED REGION ID(methods1414068524245) ENABLED START*/
// Add additional options here
/*PROTECTED REGION END*/
} // namespace alica
