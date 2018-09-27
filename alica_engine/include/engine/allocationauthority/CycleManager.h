#pragma once
//#define CM_DEBUG

#include "engine/AlicaClock.h"
#include "engine/allocationauthority/AllocationDifference.h"
#include "supplementary/AgentID.h"

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
struct AllocationAuthorityInfo;
class Assignment;
class AlicaEngine;

/**
 * Responsibile for detecting cycles in assignment updates and reactions to these
 */
class CycleManager
{
  public:
    CycleManager(AlicaEngine* ae, RunningPlan* p);
    virtual ~CycleManager();
    void update();
    bool isOverridden() const;
    bool setAssignment();
    bool mayDoUtilityCheck();
    void setNewAllocDiff(AllocationDifference* aldif);
    void setNewAllocDiff(std::shared_ptr<Assignment> oldAss, std::shared_ptr<Assignment> newAss, AllocationDifference::Reason reas);
    void handleAuthorityInfo(std::shared_ptr<AllocationAuthorityInfo> aai);
    bool needsSending();
    void sent();
    bool haveAuthority();

  private:
    enum CycleState
    {
        observing,
        overridden,
        overriding
    };
    bool detectAllocationCycle();

    AlicaEngine* ae;
    std::mutex allocationHistoryMutex;
    supplementary::SystemConfig* sc;
    int maxAllocationCycles;
    bool enabled;
    std::vector<AllocationDifference*> allocationHistory;
    PlanRepository* pr;
    int newestAllocationDifference;
    const supplementary::AgentID* myID;

    AlicaTime overrideTimestamp;
    double intervalIncFactor;
    double intervalDecFactor;
    AlicaTime minimalOverrideTimeInterval;
    AlicaTime maximalOverrideTimeInterval;
    AlicaTime overrideShoutInterval;
    AlicaTime overrideWaitInterval;
    AlicaTime overrideShoutTime;
    int historySize;
    CycleState state;
    RunningPlan* rp;
    std::shared_ptr<AllocationAuthorityInfo> fixedAllocation;
};

} // namespace alica
