#include "VariableHandling/constraints/Lvl31524452836022Constraints.h"
/*PROTECTED REGION ID(ch1524452836022) ENABLED START*/

#include <assert.h>
#include <autodiff/AutoDiff.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
using autodiff::TermPtr;

/*PROTECTED REGION END*/

namespace alica
{
// Plan:Lvl3
/*
 * Tasks:
 * - EP:1524452836024 : DefaultTask (1225112227903)
 *
 * States:
 * - NewState (1524452836023)
 *
 * Vars:
 * - L3A (1524453054226)
 * - L3B (1524453060294)
 */
/**
 * RuntimeCondition - (Name): NewRuntimeCondition
 * (ConditionString): Lvl3 Runtime Condition
 * Static Variables: L3A L3B
 * Domain Variables:
 * forall agents in Lvl3 let v = [X, Y]
 *
 */
void Constraint1524452937477::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(cc1524452937477) ENABLED START*/
    assert(c->getStaticVars().size() == 2);
    assert(c->getAgentVars().size() == 1);
    assert(c->getAgentVars()[0].getVars().size() == 2);
    TermPtr l1a = static_cast<autodiff::Variable*>(c->getStaticVars()[0]);
    TermPtr l1b = static_cast<autodiff::Variable*>(c->getStaticVars()[1]);

    autodiff::Variable* xv = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[0]);
    autodiff::Variable* yv = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[1]);

    TermPtr x = xv;
    TermPtr y = yv;
    xv->editRange().intersect(-5, 5);
    yv->editRange().intersect(-5, 5);

    TermPtr constraint = x * l1a < x->getOwner()->constant(0.0);
    constraint = constraint & (y * l1b > x->getOwner()->constant(0.0));

    c->setConstraint(constraint);
    TermPtr utility = l1a * l1a + l1b * l1b;

    c->setUtility(utility);
    c->setUtilitySufficiencyThreshold(10.0);

    /*PROTECTED REGION END*/
}

// State: NewState
} // namespace alica
