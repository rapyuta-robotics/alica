#include "VariableHandling/constraints/Lvl11524452759599Constraints.h"
/*PROTECTED REGION ID(ch1524452759599) ENABLED START*/
// Add additional using directives here
#include <assert.h>
#include <autodiff/AutoDiff.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
using autodiff::TermPtr;

/*PROTECTED REGION END*/

namespace alica
{
// Plan:Lvl1
/*
 * Tasks:
 * - EP:1524452759601 : DefaultTask (1225112227903)
 *
 * States:
 * - NewState (1524452759600)
 * - BeforeTrans (1524453481856)
 *
 * Vars:
 * - L1A (1524453326397)
 * - L1B (1524453331530)
 * - L1C (1524453336548)
 */
/**
 * RuntimeCondition - (Name): NewRuntimeCondition
 * (ConditionString): Lvl1 Runtime Condition
 * Static Variables: L1A L1B
 * Domain Variables:
 *
 */
void Constraint1524453470580::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(cc1524453470580) ENABLED START*/
    autodiff::Variable* l1av = static_cast<autodiff::Variable*>(c->getStaticVars()[0]);
    autodiff::Variable* l1bv = static_cast<autodiff::Variable*>(c->getStaticVars()[1]);

    TermPtr l1a = l1av;
    TermPtr l1b = l1bv;

    assert(c->getStaticVars().size() == 2);
    assert(c->getAgentVars().empty());

    l1av->editRange().intersect(-200.0, -100.0);
    l1bv->editRange().intersect(100.0, 200.0);

    TermPtr constraint = l1a + l1b < l1b->getOwner()->constant(10.0);
    constraint &= l1a + l1b > l1b->getOwner()->constant(-10.0);

    c->setConstraint(constraint);

    TermPtr utility = l1b->getOwner()->constant(1);
    c->setUtility(utility);
    c->setUtilitySufficiencyThreshold(10.0);

    /*PROTECTED REGION END*/
}

// State: NewState
// State: BeforeTrans
/**
 * Transition:
 * - Name: MISSING_NAME
 * - Comment:
 * - ConditionString:
 *
 *
 * AbstractPlans in State:
 * Static Variables:
 * Domain Variables:
 * forall agents in MISSING_NAME let v = [X, Y]
 */
void Constraint1524453491764::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(cc1524453491764) ENABLED START*/
    assert(c->getStaticVars().empty());
    assert(c->getAgentVars().size() == 2);

    autodiff::Variable* x1v = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[0]);
    autodiff::Variable* y1v = static_cast<autodiff::Variable*>(c->getAgentVars()[0].getVars()[1]);

    autodiff::Variable* x2v = static_cast<autodiff::Variable*>(c->getAgentVars()[1].getVars()[0]);
    autodiff::Variable* y2v = static_cast<autodiff::Variable*>(c->getAgentVars()[1].getVars()[1]);

    TermPtr x1 = x1v;
    TermPtr y1 = y1v;
    TermPtr x2 = x2v;
    TermPtr y2 = y2v;

    TermPtr constraint = (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) > x1->getOwner()->constant(10000.0);
    constraint &= x1 > y1;
    constraint &= x2 > y2;

    c->setConstraint(constraint);
    TermPtr utility = x1->getOwner()->constant(1.0);

    c->setUtility(utility);
    /*PROTECTED REGION END*/
}
} // namespace alica
