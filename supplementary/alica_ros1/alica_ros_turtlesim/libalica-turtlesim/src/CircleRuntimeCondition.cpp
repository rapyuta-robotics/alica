#include "CircleRuntimeCondition.h"

#include <autodiff/AutoDiff.h>
#include <engine/RunningPlan.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
#include <engine/model/Task.h>

namespace turtlesim
{

bool CircleRuntimeCondition::evaluate(std::shared_ptr<alica::RunningPlan> rp, const alica::Blackboard* globalBlackboard)
{
    return true;
}

std::shared_ptr<CircleRuntimeCondition> CircleRuntimeCondition::create(alica::ConditionContext&)
{
    return std::make_shared<CircleRuntimeCondition>();
}

void CircleRuntimeConditionConstraint::getConstraint(std::shared_ptr<alica::ProblemDescriptor> c, std::shared_ptr<alica::RunningPlan> rp)
{
    using autodiff::TermPtr;
    autodiff::TermHolder* h = static_cast<autodiff::TermHolder*>(c->getContext());
    TermPtr constraint = h->trueConstant();

    // set utility as constant which means result meet only constraints.
    // if you set non constant, ALICA will maximize the utility.
    c->setUtility(h->constant(1));

    // iterate through agents
    const int agentNum = static_cast<int>(c->getAgentVars().size());
    for (int i = 0; i < agentNum; ++i) {
        // get variables which is define in editor->Runtime condition->Quantifiers
        autodiff::Variable* x = static_cast<autodiff::Variable*>(c->getAgentVars()[i].getVars()[0]);
        autodiff::Variable* y = static_cast<autodiff::Variable*>(c->getAgentVars()[i].getVars()[1]);

        // check agent is leader
        const alica::EntryPoint* agentEntrypoint = rp->getAssignment().getEntryPointOfAgent(c->getAgentVars()[i].getId());
        if (!agentEntrypoint) {
            // Engine doesn't guarantee consistent plan view during this function execution. A null entry
            // point means that the plan status changed between the time this function was invoked
            // and entry point was requested. Don't do anything now and ignore further constraints building.
            return;
        }

        const bool isLeader = agentEntrypoint->getTask()->getName() == "Leader";

        // set range of variables
        // 0~10 is turtlesim default area size -> center is (5,5)
        x->editRange().intersect(0, 10);
        y->editRange().intersect(0, 10);
        autodiff::TVec<2> pos(x, y);

        // place turtle between radiusMin and radiusMax
        constexpr float tolerance = 0.1;
        constexpr float radius = 2.5;
        constexpr float radiusMin = radius - tolerance;
        constexpr float radiusMax = radius + tolerance;
        const autodiff::TVec<2> center(h->constant(5.0), h->constant(5.0));

        if (isLeader) // leader goes to center
        {
            constraint &= autodiff::equals(pos, center, tolerance);
        } else // follower keep distance from center and other followers
        {
            // keep distance from center
            TermPtr distToCenterSqr = autodiff::distanceSqr(pos, center);
            constraint &= (distToCenterSqr < h->constant(radiusMax * radiusMax));
            constraint &= (distToCenterSqr > h->constant(radiusMin * radiusMin));

            // keep distance among followers
            for (int j = i + 1; j < agentNum; ++j) {
                // check agent is leader
                // another way to check agent is Leader
                const alica::EntryPoint* otherAgentEntrypoint = rp->getAssignment().getEntryPointOfAgent(c->getAgentVars()[j].getId());
                if (!otherAgentEntrypoint) {
                    // Engine doesn't guarantee consistent plan view during this function execution. A null entry
                    // point means that the plan status changed between the time this function was invoked
                    // and entry point was requested. Don't do anything now and ignore further constraints building.
                    return;
                }

                const bool isOthersLeader = (otherAgentEntrypoint->getId() == 4346694000146342467);
                if (!isOthersLeader) {
                    autodiff::Variable* xOther = static_cast<autodiff::Variable*>(c->getAgentVars()[j].getVars()[0]);
                    autodiff::Variable* yOther = static_cast<autodiff::Variable*>(c->getAgentVars()[j].getVars()[1]);
                    const autodiff::TVec<2> posOther(xOther, yOther);
                    TermPtr distToOtherSqr = autodiff::distanceSqr(pos, posOther);

                    // calculate distance for neighbors
                    float disBetweenFollowers = 2.0 * radius * sin(M_PI / (float) (agentNum - 1)) - tolerance;
                    constraint &= (distToOtherSqr > h->constant(disBetweenFollowers * disBetweenFollowers));
                }
            }
        }
    }

    c->setConstraint(constraint);
}

std::shared_ptr<CircleRuntimeConditionConstraint> CircleRuntimeConditionConstraint::create(alica::ConstraintContext&)
{
    return std::make_shared<CircleRuntimeConditionConstraint>();
}

} // namespace turtlesim
