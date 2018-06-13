#pragma once
#include <engine/Assignment.h>
#include <engine/RunningPlan.h>

namespace alica
{

class RunningPlan;

class ReadLockedPlanPointer
{
public:
    ReadLockedPlanPointer(const RunningPlan* rp)
            : _rp(rp)
            , _lck(rp->getReadLock())
    {
    }
    void moveUp()
    {
        if (_rp) {
            _rp = _rp->getParent();
            _lck = _rp->getReadLock();
        }
    }
    RunningPlan::ScopedReadLock& getLock() { return _lck; }
    const RunningPlan* get() const { return _rp; }

private:
    const RunningPlan* _rp;
    RunningPlan::ScopedReadLock _lck;
};

class SafeAssignmentView : public AssignmentView
{
public:
    SafeAssignmentView()
            : AssignmentView()
            , _lck()
    {
    }

    SafeAssignmentView(const Assignment* a, int epIdx, ReadLockedPlanPointer&& rlpp)
            : AssignmentView(a, epIdx)
            , _lck(std::move(rlpp.getLock()))
    {
    }

private:
    RunningPlan::ScopedReadLock _lck;
};

class ThreadSafePlanInterface
{
public:
    ThreadSafePlanInterface()
            : _rp(nullptr)
    {
    }
    explicit ThreadSafePlanInterface(const RunningPlan* rp)
            : _rp(rp)
    {
    }
    bool mapsTo(const RunningPlan* rp) const { return _rp == rp; }

    SafeAssignmentView agentsInEntryPointOfHigherPlan(const EntryPoint* ep) const;

    SafeAssignmentView agentsInEntryPoint(const EntryPoint* ep) const;

    const EntryPoint* getParentEntryPoint(const std::string& taskName) const;

    const EntryPoint* getHigherEntryPoint(const std::string& planName, const std::string& taskName) const;

    const AbstractPlan* getActivePlan() const;

private:
    const RunningPlan* _rp;
};
}
