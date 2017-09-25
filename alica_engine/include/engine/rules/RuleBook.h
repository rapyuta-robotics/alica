#pragma once

//#define RULE_debug

#include "engine/PlanChange.h"

#include <memory>

namespace alica
{
class ISyncModul;
class IPlanSelector;
class Logger;
class ITeamObserver;
class ITeamManager;
class RunningPlan;
class Plan;
class EntryPoint;
class Transition;
class State;
class EntryPoint;
class ConditionStore;
class StateCollection;
class CycleManager;
class UtilityFunction;
class AlicaEngine;

/**
 * Defines the operational semantics of the used ALICA dialect.
 */
class RuleBook
{
  public:
    RuleBook(AlicaEngine *ae);
    virtual ~RuleBook();
    bool isChangeOccured() const;
    void setChangeOccured(bool changeOccured);
    PlanChange visit(std::shared_ptr<RunningPlan> r);
    PlanChange updateChange(PlanChange cur, PlanChange update);
    std::shared_ptr<RunningPlan> initialisationRule(Plan *masterPlan);

  protected:
    AlicaEngine *ae;
    ITeamObserver *to;
    ITeamManager *tm;
    ISyncModul *sm;
    int maxConsecutiveChanges;
    IPlanSelector *ps;
    Logger *log;
    bool changeOccured;
    PlanChange synchTransitionRule(std::shared_ptr<RunningPlan> r);
    PlanChange transitionRule(std::shared_ptr<RunningPlan> r);
    PlanChange topFailRule(std::shared_ptr<RunningPlan> r);
    PlanChange allocationRule(std::shared_ptr<RunningPlan> r);
    PlanChange authorityOverrideRule(std::shared_ptr<RunningPlan> r);
    PlanChange planAbortRule(std::shared_ptr<RunningPlan> r);
    PlanChange planRedoRule(std::shared_ptr<RunningPlan> r);
    PlanChange planReplaceRule(std::shared_ptr<RunningPlan> r);
    PlanChange planPropagationRule(std::shared_ptr<RunningPlan> r);
    PlanChange dynamicAllocationRule(std::shared_ptr<RunningPlan> r);
};
}
