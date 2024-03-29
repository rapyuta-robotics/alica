#include "Conditions.h"
#include <engine/BasicBehaviour.h>

namespace alica_standard_library
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

bool AlwaysTrueCondition(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return true;
}

bool AlwaysFalseCondition(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return false;
}

bool IsChildSuccess(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    std::string childName = bb.get<std::string>("childName");

    for (const alica::RunningPlan* child : rp->getChildren()) {
        std::string rpName = rp->isBehaviour() ? child->getBasicBehaviour()->getName() : child->getActivePlan()->getName();
        if (rpName == childName) {
            return isSuccess(child);
        }
    }
    return false;
}

template <typename T>
bool IsEqual(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    return bb.get<T>("left") == bb.get<T>("right");
}

template <typename T>
bool IsNotEqual(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return !IsEqual<T>(input, rp, gb);
}

template <typename T>
bool IsLessThan(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    return bb.get<T>("left") < bb.get<T>("right");
}

template <typename T>
bool IsLessThanOrEqual(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    return bb.get<T>("left") <= bb.get<T>("right");
}

template <typename T>
bool IsGreaterThan(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return !IsLessThanOrEqual<T>(input, rp, gb);
}

template <typename T>
bool IsGreaterThanOrEqual(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    return !IsLessThan<T>(input, rp, gb);
}

bool IsChildFailure(const alica::Blackboard* input, const alica::RunningPlan* rp, const alica::Blackboard* gb)
{
    alica::LockedBlackboardRO bb(*input);
    std::string childName = bb.get<std::string>("childName");

    for (const alica::RunningPlan* child : rp->getChildren()) {
        std::string rpName = rp->isBehaviour() ? child->getBasicBehaviour()->getName() : child->getActivePlan()->getName();
        if (rpName == childName) {
            return isFailure(child);
        }
    }
    return false;
}
} /* namespace alica_standard_library */
