#include "engine/PlanInterface.h"

#include <assert.h>

namespace alica
{

using SafeRPPointer = std::pair<const RunningPlan, RunningPlan::ScopedReadLock>;

SafeAssignmentView PlanInterface::agentsInEntryPointOfHigherPlan(const EntryPoint* ep) const
{
    assert(ep);
    if (ep == nullptr) {
        SafeAssignmentView();
    }
    ReadLockedPlanPointer cur(_runningPlan);
    cur.moveUp();
    while (cur.get() != nullptr) {
        assert(cur.get()->getActivePlan());
        if (cur.get()->getActivePlan() == ep->getPlan()) {
            return SafeAssignmentView(cur.get()->getAssignment(), ep->getIndex(), std::move(cur));
        }
        cur.moveUp();
    }
    return SafeAssignmentView();
}

SafeAssignmentView PlanInterface::agentsInEntryPoint(const EntryPoint* ep) const
{
    assert(ep);
    if (ep == nullptr) {
        return SafeAssignmentView();
    }
    ReadLockedPlanPointer cur(_runningPlan);
    cur.moveUp();
    if (cur.get() != nullptr) {
        return SafeAssignmentView(cur->getAssignment(), ep->getIndex(), std::move(cur));
    }
    return SafeAssignmentView();
}

const EntryPoint* PlanInterface::getParentEntryPoint(const std::string& taskName)
{
    const Plan* parentPlan = nullptr;
    {
        ReadLockedPlanPointer cur(_runningPlan);
        cur.moveUp();
        if (cur.get() == nullptr) {
            return nullptr;
        }
        parentPlan = static_cast<const Plan*>(cur.get()->getActivePlan());
    }
    for (const EntryPoint* e : parentPlan->getEntryPoints()) {
        if (e->getTask()->getName() == taskName) {
            return e;
        }
    }
    return nullptr;
}

const EntryPoint* PlanInterface::getHigherEntryPoint(const std::string& planName, const std::string& taskName)
{
    const Plan* parentPlan = nullptr;
    {
        ReadLockedPlanPointer cur(_runningPlan);
        cur.moveUp();
        while (cur.get() != nullptr) {
            if (cur.get()->getActivePlan()->getName() == planName) {
                parentPlan = static_cast<const Plan*>(cur.get()->getActivePlan());
                break;
            }
            cur.moveUp();
        }
    }
    for (const EntryPoint* e : parentPlan->getEntryPoints()) {
        if (e->getTask()->getName() == taskName) {
            return e;
        }
    }
    return nullptr;
}

const AbstractPlan* PlanInterface::getActivePlan() const
{
    RunningPlan::ScopedReadLock lck(_runningPlan->getReadLock());
    return _runningPlan->getActivePlan();
}
}