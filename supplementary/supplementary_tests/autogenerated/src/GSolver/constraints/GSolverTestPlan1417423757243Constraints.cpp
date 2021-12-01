#include "GSolver/constraints/GSolverTestPlan1417423757243Constraints.h"
/*PROTECTED REGION ID(ch1417423757243) ENABLED START*/
#include <autodiff/AutoDiff.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <iostream>

using namespace autodiff;
/*PROTECTED REGION END*/

namespace alica
{
// Plan:GSolverTestPlan
/*
 * Tasks:
 * - EP:1417423777546 : DefaultTask (1225112227903)
 *
 * States:
 * - SolverState (1417423777544)
 *
 * Vars:
 * - X (1417444589341)
 * - Y (1417444593509)
 */
/**
 * RuntimeCondition - (Name): NewRuntimeCondition
 * (ConditionString):
 * Static Variables: X Y
 * Domain Variables:
 *
 */
void Constraint1417424512343::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(cc1417424512343) ENABLED START*/
    // Proteced
    autodiff::Variable* xv = dynamic_cast<autodiff::Variable*>(c->getStaticVars()[0]);
    if (xv == nullptr) {
        std::cout << "Constraint1417424512343::getConstraint !!!!!!!!!!!!!!!!!!!!! error" << std::endl;
        return;
    }
    xv->editRange().intersect(0.0, 10000.0);

    autodiff::Variable* yv = dynamic_cast<autodiff::Variable*>(c->getStaticVars()[1]);
    if (yv == nullptr) {
        std::cout << "Constraint1417424512343::getConstraint !!!!!!!!!!!!!!!!!!!!! error" << std::endl;
        return;
    }
    yv->editRange().intersect(0.0, 10000.0);

    TermPtr x = xv;
    TermPtr y = yv;

    TermPtr constraint = x->getOwner()->trueConstant();
    constraint = constraint & (x->getOwner()->constant(5000) > x);
    constraint = constraint & (x->getOwner()->constant(4000) < x);

    constraint = constraint & (x->getOwner()->constant(8000) > y);
    constraint = constraint & (x->getOwner()->constant(7000) < y);
    c->setConstraint(constraint);

    TermPtr utility = x->getOwner()->constant(1.0);
    c->setUtility(utility);
    c->setUtilitySufficiencyThreshold(1.0);

    /*PROTECTED REGION END*/
}

// State: SolverState
} // namespace alica
