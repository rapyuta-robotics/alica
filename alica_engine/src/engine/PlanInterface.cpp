#include "engine/PlanInterface.h"
#include "engine/model/Task.h"
#include <assert.h>

namespace alica
{

using SafeRPPointer = std::pair<const RunningPlan, RunningPlan::ScopedReadLock>;

SafeAssignmentView ThreadSafePlanInterface::agentsInEntryPointOfHigherPlan(const EntryPoint* ep) const
{
    assert(ep);
    if (ep == nullptr) {
        SafeAssignmentView();
    }
    ReadLockedPlanPointer cur(_rp);
    cur.moveUp();
    while (cur) {
        assert(cur->getActivePlan());
        if (cur->getActivePlan() == ep->getPlan()) {
            return SafeAssignmentView(&cur->getAssignment(), ep->getIndex(), std::move(cur));
        }
        cur.moveUp();
    }
    return SafeAssignmentView();
}

SafeAssignmentView ThreadSafePlanInterface::agentsInEntryPoint(const EntryPoint* ep) const
{
    assert(ep);
    if (ep == nullptr) {
        return SafeAssignmentView();
    }
    ReadLockedPlanPointer cur(_rp);
    cur.moveUp();
    if (cur) {
        return SafeAssignmentView(&cur->getAssignment(), ep->getIndex(), std::move(cur));
    }
    return SafeAssignmentView();
}

const EntryPoint* ThreadSafePlanInterface::getParentEntryPoint(const std::string& taskName) const
{
    const Plan* parentPlan = nullptr;
    {
        ReadLockedPlanPointer cur(_rp);
        cur.moveUp();
        if (!cur) {
            return nullptr;
        }
        parentPlan = static_cast<const Plan*>(cur->getActivePlan());
    }
    for (const EntryPoint* e : parentPlan->getEntryPoints()) {
        if (e->getTask()->getName() == taskName) {
            return e;
        }
    }
    return nullptr;
}

const EntryPoint* ThreadSafePlanInterface::getHigherEntryPoint(const std::string& planName, const std::string& taskName) const
{
    const Plan* parentPlan = nullptr;
    {
        ReadLockedPlanPointer cur(_rp);
        cur.moveUp();
        while (cur) {
            if (cur->getActivePlan()->getName() == planName) {
                parentPlan = static_cast<const Plan*>(cur->getActivePlan());
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

const AbstractPlan* ThreadSafePlanInterface::getActivePlan() const
{
    RunningPlan::ScopedReadLock lck(_rp->getReadLock());
    return _rp->getActivePlan();
}
}