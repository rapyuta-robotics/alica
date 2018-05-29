#pragma once
//#define CM_DEBUG

#include "engine/allocationauthority/AllocationDifference.h"
#include "engine/AlicaClock.h"
#include "supplementary/AgentID.h"

#include <vector>
#include <thread>
#include <mutex>
using namespace std;
namespace supplementary {
class SystemConfig;
}
namespace alica {
class RunningPlan;
class PlanRepository;
struct AllocationAuthorityInfo;
class Assignment;
class AlicaEngine;

/**
 * Responsibile for detecting cycles in assignment updates and reactions to these
 */
class CycleManager {
public:
    CycleManager(AlicaEngine* ae, RunningPlan* p);
    virtual ~CycleManager();
    void update();
    bool isOverridden() const;
    bool setAssignment();
    bool mayDoUtilityCheck();
    void setNewAllocDiff(AllocationDifference* aldif);
    void setNewAllocDiff(
            shared_ptr<Assignment> oldAss, shared_ptr<Assignment> newAss, AllocationDifference::Reason reas);
    void handleAuthorityInfo(shared_ptr<AllocationAuthorityInfo> aai);
    bool needsSending();
    void sent();
    bool haveAuthority();

protected:
    AlicaEngine* ae;
    mutex allocationHistoryMutex;
    supplementary::SystemConfig* sc;
    int maxAllocationCycles;
    bool enabled;
    vector<AllocationDifference*> allocationHistory;
    PlanRepository* pr;
    int newestAllocationDifference;
    const supplementary::AgentID* myID;
    enum CycleState { observing, overridden, overriding };
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
    shared_ptr<AllocationAuthorityInfo> fixedAllocation;
    bool detectAllocationCycle();
};

}  // namespace alica
