#pragma once

#include "engine/PlanChange.h"

#include <any>
#include <memory>
#include <unordered_map>
#include <yaml-cpp/yaml.h>

namespace alica
{
class SyncModule;
class PlanSelector;
class RunningPlan;
class Plan;
class PlanBase;
class EntryPoint;
class Transition;
class State;
class EntryPoint;
class ConditionStore;
class CycleManager;
class UtilityFunction;
class TeamObserver;
class TeamManager;
class Blackboard;
class ConfigChangeListener;
class PlanRepository;

/**
 * Defines the operational semantics of the used ALICA dialect.
 */
class RuleBook
{
public:
    RuleBook(ConfigChangeListener& configChangeListener, SyncModule& syncModule, TeamObserver& teamObserver, const TeamManager& teamManager,
            const PlanRepository& planRepository, PlanBase* pb);
    ~RuleBook();
    bool hasChangeOccurred() const { return _changeOccurred; }
    PlanChange visit(RunningPlan& r);
    PlanChange updateChange(PlanChange cur, PlanChange update);
    RunningPlan* initialisationRule(const Plan* masterPlan);
    void resetChangeOccurred() { _changeOccurred = false; }
    PlanSelector* getPlanSelector() const { return _ps.get(); }
    void reload(const YAML::Node& config);
    void init(std::shared_ptr<const Blackboard> globalBlackboard);

private:
    static constexpr const char* LOGNAME = "Rulebook";

    SyncModule& _syncModule;
    const TeamManager& _teamManager;
    std::unique_ptr<PlanSelector> _ps;
    PlanBase* _pb;

    int _maxConsecutiveChanges;
    bool _autoFailureHandlingEnabled;
    bool _changeOccurred;
    std::shared_ptr<const Blackboard> _globalBlackboard;

    PlanChange synchTransitionRule(RunningPlan& rp);
    PlanChange transitionRule(RunningPlan& r);
    PlanChange topFailRule(RunningPlan& r);
    PlanChange allocationRule(RunningPlan& r);
    PlanChange authorityOverrideRule(RunningPlan& r);
    PlanChange planAbortRule(RunningPlan& r);
    PlanChange planRedoRule(RunningPlan& r);
    PlanChange planReplaceRule(RunningPlan& r);
    PlanChange planPropagationRule(RunningPlan& r);
    PlanChange dynamicAllocationRule(RunningPlan& r);
};
} // namespace alica
