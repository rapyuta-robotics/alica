#pragma once
//#define CM_DEBUG

#include "engine/AlicaClock.h"
#include "engine/Types.h"
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
class ConfigChangeListener;
class TeamManager;

/**
 * Responsibile for detecting cycles in assignment updates and reactions to these
 */
class CycleManager
{
public:
    CycleManager(ConfigChangeListener& configChangeListener, const AlicaClock& clock, const TeamManager& teamManager, const PlanRepository& planRepository,
            RunningPlan* p);
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
    static constexpr const char* LOGNAME = "CycleManager";

    enum CycleState
    {
        observing,
        overridden,
        overriding
    };
    bool detectAllocationCycle();

    const AlicaClock& _clock;
    const TeamManager& _teamManager;
    const PlanRepository& _planRepository;
    ConfigChangeListener& _configChangeListener;
    std::vector<AllocationDifference> _allocationHistory;
    int _newestAllocationDifference;
    int _maxAllocationCycles;
    uint64_t _reloadCallbackId;
    bool _enabled;
    AgentId _myID;
    AlicaTime _overrideTimestamp;
    double _intervalIncFactor;
    double _intervalDecFactor;
    AlicaTime _minimalOverrideTimeInterval;
    AlicaTime _maximalOverrideTimeInterval;
    AlicaTime _overrideShoutInterval;
    AlicaTime _overrideWaitInterval;
    AlicaTime _overrideShoutTime;
    CycleState _state;
    RunningPlan* _runningPlan;
    AllocationAuthorityInfo _fixedAllocation;
};

} // namespace alica
