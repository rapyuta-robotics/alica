#pragma once
//#define CM_DEBUG

#include "engine/AlicaClock.h"
#include "engine/allocationauthority/AllocationDifference.h"
#include "engine/containers/AllocationAuthorityInfo.h"

#include <mutex>
#include <thread>
#include <vector>
#include <yaml-cpp/yaml.h>

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
    void reload(const YAML::Node& config);

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
    int _maxAllocationCycles;
    bool _enabled;
    alica::AgentId _myID;
    AlicaTime _overrideTimestamp;
    double _intervalIncFactor;
    double _intervalDecFactor;
    AlicaTime _minimalOverrideTimeInterval;
    AlicaTime _maximalOverrideTimeInterval;
    AlicaTime _overrideShoutInterval;
    AlicaTime _overrideWaitInterval;
    AlicaTime _overrideShoutTime;
    CycleState _state;
    RunningPlan* _rp;
    AllocationAuthorityInfo _fixedAllocation;
};

} // namespace alica
