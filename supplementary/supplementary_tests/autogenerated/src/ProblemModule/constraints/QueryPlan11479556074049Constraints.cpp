#include "ProblemModule/constraints/QueryPlan11479556074049Constraints.h"
/*PROTECTED REGION ID(ch1479556074049) ENABLED START*/

#include <autodiff/AutoDiff.h>
#include <engine/constraintmodul/ProblemDescriptor.h>

/*PROTECTED REGION END*/

namespace alica
{
// Plan:QueryPlan1
/*
 * Tasks:
 * - EP:1479556074051 : DefaultTask (1225112227903)
 *
 * States:
 * - QueryState1 (1479556074050)
 *
 * Vars:
 * - QP1X (1479556220234)
 * - QP1Y (1479556572534)
 */
/**
 * RuntimeCondition - (Name): NewRuntimeCondition
 * (ConditionString):
 * Static Variables: QP1X QP1Y
 * Domain Variables:
 * forall agents in QueryState1 let v = [X, Y, Z]
 *
 */
void Constraint1479556084493::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    /*PROTECTED REGION ID(cc1479556084493) ENABLED START*/
    autodiff::TermHolder* h = static_cast<autodiff::TermHolder*>(c->getContext());
    c->setConstraint(h->trueConstant());
    c->setUtility(h->constant(1));
    /*PROTECTED REGION END*/
}

// State: QueryState1
} // namespace alica
