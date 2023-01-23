#include "Conditions.h"

namespace utils
{

bool isSuccess(const alica::RunningPlan* rp)
{
    if (rp->isBehaviour()) {
        return rp->getStatus() == alica::PlanStatus::Success;
    } else {
        return rp->getActiveState()->isSuccessState();
    }
}

bool AnyChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard)
{
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (isSuccess(child)) {
            return true;
        }
    }
    return false;
}

bool AllChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* globalBlackboard)
{
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (!isSuccess(child)) {
            return false;
        }
    }
    // In case of a state, make sure that all children are actually running
    if (rp->getActiveTriple().state) {
        return rp->getChildren().size() >= rp->getActiveTriple().state->getConfAbstractPlanWrappers().size();
    }
    return true;
}

} /* namespace utils */
