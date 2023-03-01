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

bool isFailure(const alica::RunningPlan* rp)
{
    if (rp->isBehaviour()) {
        return rp->getStatus() == alica::PlanStatus::Failed;
    } else {
        return rp->getActiveState()->isFailureState();
    }
}

bool AnyChildFailure(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (isFailure(child)) {
            return true;
        }
    }
    return false;
}

bool AllChildFailure(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    for (const alica::RunningPlan* child : rp->getChildren()) {
        if (!isFailure(child)) {
            return false;
        }
    }
    // In case of a state, make sure that all children are actually running
    if (rp->getActiveTriple().state) {
        return rp->getChildren().size() >= rp->getActiveTriple().state->getConfAbstractPlanWrappers().size();
    }
    return true;
}

bool IsAnyChildStatusSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return false;
}

bool AlwaysTrueCond(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return true;
}

bool DefaultCondition(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return false;
}

bool IsAnyChildStatus(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    return rp->isAnyChildStatus(bb.get<alica::PlanStatus>("childStatus"));
}

bool IsAnyChildStatusFailed(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return rp->isAnyChildStatus(alica::PlanStatus::Failed);
}

bool isAnyChildTaskSuccessfull(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return rp->isAnyChildTaskSuccessful();
}

} /* namespace utils */
