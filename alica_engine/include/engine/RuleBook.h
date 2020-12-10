#pragma once

#include "engine/PlanChange.h"

#include <memory>
#include <yaml-cpp/yaml.h>

namespace alica
{
class SyncModule;
class PlanSelector;
class Logger;
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
class AlicaEngine;
class TeamObserver;
class TeamManager;

/**
 * Defines the operational semantics of the used ALICA dialect.
 */
class RuleBook
{
public:
    RuleBook(AlicaEngine* ae, PlanBase* pb);
    ~RuleBook();
    bool hasChangeOccurred() const { return _changeOccurred; }
    PlanChange visit(RunningPlan& r);
    PlanChange updateChange(PlanChange cur, PlanChange update);
    RunningPlan* initialisationRule(const Plan* masterPlan);
    void resetChangeOccurred() { _changeOccurred = false; }
    PlanSelector* getPlanSelector() const { return _ps.get(); }
    void reload(const YAML::Node& config);

private:
    const TeamManager& _tm;
    SyncModule& _sm;
    std::unique_ptr<PlanSelector> _ps;
    PlanBase* _pb;
    Logger& _log;
    int _maxConsecutiveChanges;
    bool _changeOccurred;
    AlicaEngine* _ae;

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
