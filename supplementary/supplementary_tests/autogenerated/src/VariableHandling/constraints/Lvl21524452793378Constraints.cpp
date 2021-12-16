#include "VariableHandling/constraints/Lvl21524452793378Constraints.h"
/*PROTECTED REGION ID(ch1524452793378) ENABLED START*/
#include <assert.h>
#include <autodiff/AutoDiff.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
using autodiff::TermPtr;
/*PROTECTED REGION END*/

namespace alica
{
// Plan:Lvl2
/*
 * Tasks:
 * - EntryPoint:1524452793380 : DefaultTask (1225112227903)
 * - EntryPoint:1524453238753 : AttackTask (1407153522080)
 *
 * States:
 * - NewState (1524452793379)
 * - Dummy (1524453248579)
 *
 * Vars:
 * - L2A (1524453150187)
 * - L2B (1524453155043)
 * - L2C (1524453162883)
 */
/**
 * RuntimeCondition - (Name): NewRuntimeCondition
 * (ConditionString): Lvl2 Runtime Condition
 * Static Variables: L2A L2B L2C
 * Domain Variables:
 * forall agents in Dummy let v = [X, Y]
 *
 */
void Constraint1524453266123::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(cc1524453266123) ENABLED START*/
    assert(c->getStaticVars().size() == 3);
    assert(c->getAgentVars().size() == 1);
    assert(c->getAgentVars()[0].getVars().size() == 2);

    autodiff::Variable* xv = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[0]);
    autodiff::Variable* yv = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[1]);

    xv->editRange().intersect(50.0, 150.0);
    yv->editRange().intersect(50.0, 150.0);
    TermPtr x = xv;
    TermPtr y = yv;

    TermPtr constraint = x > y;

    c->setConstraint(constraint);
    TermPtr utility = x + y;

    c->setUtility(utility);
    c->setUtilitySufficiencyThreshold(280.0);

    /*PROTECTED REGION END*/
}

// State: NewState
// State: Dummy
} // namespace alica
