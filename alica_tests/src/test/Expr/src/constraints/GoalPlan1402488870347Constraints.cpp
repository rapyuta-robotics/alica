#include "constraints/GoalPlan1402488870347Constraints.h"
/*PROTECTED REGION ID(ch1402488870347) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:GoalPlan
/*
 * Tasks:
 * - EntryPoint:1402488881800 : DefaultTask (1225112227903)
 *
 * States:
 * - Shoot (1402488881799)
 * - Miss (1402489152217)
 * - Scored (1402489192198)
 *
 * Vars:
 * - test (1403773747758)
 */
/**
 * RuntimeCondition - (Name): NewRuntimeCondition
 * (ConditionString): test
 * Static Variables: test
 * Domain Variables:
 * forall agents in Miss let v = [test]
 *
 */
void Constraint1403773741874::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(cc1403773741874) ENABLED START*/
    // Proteced
    /*PROTECTED REGION END*/
}

// State: Shoot
// State: Miss
// State: Scored
} // namespace alica
