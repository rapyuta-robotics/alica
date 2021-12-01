#include "constraints/AttackPlan1402488634525Constraints.h"
/*PROTECTED REGION ID(ch1402488634525) ENABLED START*/
// Add additional using directives here
/*PROTECTED REGION END*/

namespace alica
{
// Plan:AttackPlan
/*
 * Tasks:
 * - EntryPoint:1402488646221 : DefaultTask (1225112227903)
 *
 * States:
 * - Attack (1402488646220)
 * - Shoot (1402489396914)
 *
 * Vars:
 * - TestVar1 (1403772778288)
 * - VarTest2 (1403772797469)
 * - NewVar (1403772816953)
 * - ABC (1403772834750)
 */
// State: Attack
/**
 * Transition:
 * - Name: MISSING_NAME
 * - Comment:
 * - ConditionString:
 *
 *
 * AbstractPlans in State:
 * - AbstractPlan Name: Tackle, PlanID: 1402489318663
 * - AbstractPlan Name: AttackOpp, PlanID: 1402489351885
 * Static Variables:
 * Domain Variables:
 * forall agents in AttackPlan let v = [X, Y]
 * forall agents in Attack let v = [A, B]
 * forall agents in Shoot let v = [another one]
 * forall agents in MISSING_NAME let v = [TaskQuantifier]
 */
void Constraint1402489460549::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(cc1402489460549) ENABLED START*/
    // Proteced
    /*PROTECTED REGION END*/
}
// State: Shoot
/**
 * Transition:
 * - Name: ConditionNameShootAttack
 * - Comment:
 * - ConditionString: Some nice comment!
 *
 *
 * AbstractPlans in State:
 * - AbstractPlan Name: Attack, PlanID: 1402488848841
 * Static Variables: TestVar1 VarTest2 NewVar ABC
 * Domain Variables:
 */
void Constraint1402489462088::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(cc1402489462088) ENABLED START*/
    // Proteced
    /*PROTECTED REGION END*/
}
} // namespace alica
