#include "Plans/GSolver/constraints/GSolverTestPlan1417423757243Constraints.h"
using namespace std;
using namespace alica;
/*PROTECTED REGION ID(ch1417423757243) ENABLED START*/
#include <iostream>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include "AutoDiff.h"

using namespace autodiff;
/*PROTECTED REGION END*/

namespace alicaAutogenerated {
// Plan:GSolverTestPlan

/*
 * Tasks:
 * - EP:1417423777546 : DefaultTask (1225112227903)
 *
 * States:
 * - SolverState (1417423777544)
 *
 * Vars:
 * - X (1417424527157)
 * - Y (1417424531712)
 */

/*
 * RuntimeCondition - (Name): NewRuntimeCondition
 * (ConditionString):
 * Static Variables: [X, Y]
 * Domain Variables:

 */
void Constraint1417424512343::getConstraint(shared_ptr<ProblemDescriptor> c, shared_ptr<RunningPlan>) {
    /*PROTECTED REGION ID(cc1417424512343) ENABLED START*/
    // Proteced
    shared_ptr<autodiff::Term> x = dynamic_pointer_cast<autodiff::Variable>(c->getStaticVars()->at(0));
    if (!x.operator bool()) {
        cout << "Constraint1417424512343::getConstraint !!!!!!!!!!!!!!!!!!!!! error" << endl;
    }
    c->getStaticRanges()->at(0).at(0) = 0;
    c->getStaticRanges()->at(0).at(1) = 10000;

    shared_ptr<autodiff::Term> y = dynamic_pointer_cast<autodiff::Variable>(c->getStaticVars()->at(1));
    if (!y.operator bool()) {
        cout << "Constraint1417424512343::getConstraint !!!!!!!!!!!!!!!!!!!!! error" << endl;
    }
    c->getStaticRanges()->at(1).at(0) = 0;
    c->getStaticRanges()->at(1).at(1) = 10000;

    shared_ptr<autodiff::Term> constraint = autodiff::Term::TRUE;
    constraint = constraint & (TermBuilder::constant(5000) > x);
    constraint = constraint & (TermBuilder::constant(4000) < x);

    constraint = constraint & (TermBuilder::constant(8000) > y);
    constraint = constraint & (TermBuilder::constant(7000) < y);
    c->setConstraint(constraint);

    shared_ptr<autodiff::Term> utility = make_shared<autodiff::Constant>(1);
    c->setUtility(utility);
    c->setUtilitySufficiencyThreshold(1.0);

    /*PROTECTED REGION END*/
}

// State: SolverState

// State: SolverState

}  // namespace alicaAutogenerated
