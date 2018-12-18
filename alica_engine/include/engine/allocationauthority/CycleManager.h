#pragma once
//#define CM_DEBUG

#include "engine/AlicaClock.h"
#include "engine/allocationauthority/AllocationDifference.h"
#include "engine/containers/AllocationAuthorityInfo.h"
#include "essentials/AgentID.h"

#include <mutex>
#include <thread>
#include <vector>

namespace supplementary
{
class SystemConfig;
}
namespace alica
{
class RunningPlan;
class PlanRepository;
class Assignment;
class AlicaEngine;

/**
 * Responsibile for detecting cycles in assignment updates and reactions to these
 */
class CycleManager
{
public:
    CycleManager(AlicaEngine* ae, RunningPlan* p);
    ~CycleManager();
    void update();
    bool isOverridden() const;
    bool applyAssignment();
    bool mayDoUtilityCheck() const { return _state != CycleState::overridden; }

    AllocationDifference& editNextDifference();
    void setNewAllocDiff(const Assignment& oldAssignment, const Assignment& newAssignment, AllocationDifference::Reason reason);
    void handleAuthorityInfo(const AllocationAuthorityInfo& aai);
    bool needsSending() const;
    void sent();
    bool haveAuthority() const { return _state == CycleState::overriding; }

private:
    enum CycleState
    {
        observing,
        overridden,
        overriding
    };
    bool detectAllocationCycle();

    AlicaEngine* _ae;
    std::vector<AllocationDifference> _allocationHistory;
    int _newestAllocationDifference;
    int maxAllocationCycles;
    bool enabled;

    AgentIDConstPtr myID;

    AlicaTime overrideTimestamp;
    double intervalIncFactor;
    double intervalDecFactor;
    AlicaTime minimalOverrideTimeInterval;
    AlicaTime maximalOverrideTimeInterval;
    AlicaTime overrideShoutInterval;
    AlicaTime overrideWaitInterval;
    AlicaTime overrideShoutTime;
    CycleState _state;
    RunningPlan* rp;
    AllocationAuthorityInfo _fixedAllocation;
};

} // namespace alica
