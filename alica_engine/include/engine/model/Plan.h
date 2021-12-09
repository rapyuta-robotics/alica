#pragma once

#include "AbstractPlan.h"
#include "engine/Types.h"
#include "engine/BasicPlan.h"
#include "engine/util/LRUCache.h"

namespace alica
{

class EntryPoint;
class FailureState;
class SuccessState;
class PostCondition;
class State;
class Synchronisation;
class Transition;
class ExpressionHandler;
class PlanFactory;
class PreCondition;
class RuntimeCondition;
class AlicaEngine;

/**
 * An ALICA plan
 */
class Plan : public AbstractPlan
{
public:
    Plan(AlicaEngine* ae, int64_t id);
    virtual ~Plan();

    const EntryPoint* getEntryPointTaskID(int64_t taskID) const;
    const EntryPoint* getEntryPointByID(int64_t epID, int64_t dynamicID = 0) const;
    const State* getStateByID(int64_t stateID) const;

    const EntryPointGrp& getEntryPoints() const { return _allEntryPoints; }

    const StateGrp& getStates() const { return _states; }
    const FailureStateGrp& getFailureStates() const { return _failureStates; }
    const SuccessStateGrp& getSuccessStates() const { return _successStates; }

    int getMaxCardinality() const { return _maxCardinality; }
    int getMinCardinality() const { return _minCardinality; }
    bool isMasterPlan() const { return _masterPlan; }

    UtilityFunction* getUtilityFunction() const { return _utilityFunction.get(); }
    double getUtilityThreshold() const { return _utilityThreshold; }

    const TransitionGrp& getTransitions() const { return _transitions; }
    const SynchronisationGrp& getSynchronisations() const { return _synchronisations; }
    const RuntimeCondition* getRuntimeCondition() const { return _runtimeCondition; }
    const PreCondition* getPreCondition() const { return _preCondition; }

    int getFrequency() const { return _frequency; }

    std::string toString(std::string indent = "") const;

    // TODO: get rid of const
    void computeDynamicEntryPoints(const Configuration* configuration) const;

private:
    friend ModelFactory;
    friend PlanFactory;
    friend ExpressionHandler; // TODO: get rid of this
    void setEntryPoints(const std::vector<EntryPoint*>& entryPoints);
    void setFailureStates(const FailureStateGrp& failurePoints);
    void setSuccessStates(const SuccessStateGrp& succesPoints);
    void setMaxCardinality(int maxCardinality);
    void setMinCardinality(int minCardinality);
    void setMasterPlan(bool isMasterPlan);
    void setPostCondition(const PostCondition* postCondition);
    void setStates(const StateGrp& states);
    void setSynchronisations(const SynchronisationGrp &synchronisations);
    void setTransitions(const TransitionGrp& transitions);
    void setRuntimeCondition(RuntimeCondition* runtimeCondition);
    void setPreCondition(PreCondition* preCondition);

    int _minCardinality;
    int _maxCardinality;
    EntryPointGrp _entryPoints;
    mutable EntryPointGrp _allEntryPoints;
    StateGrp _states;
    SuccessStateGrp _successStates;
    FailureStateGrp _failureStates;
    SynchronisationGrp _synchronisations;
    TransitionGrp _transitions;

    AlicaEngine* _ae;
    /**
     * This plan's Utility function
     */
    // TODO: change shared to unique ptr (this requires a change to autogeneration templates)
    std::shared_ptr<UtilityFunction> _utilityFunction;
    /**
     * The utility threshold, the higher, the less likely dynamic changes are.
     */
    double _utilityThreshold;
    /**
     *  Whether this plan is marked as a MasterPlan.
     */
    bool _masterPlan;
    /**
     * This behaviour's runtime condition.
     */
    RuntimeCondition* _runtimeCondition;
    /**
     * This behaviour's precondition
     */
    PreCondition* _preCondition;
    /**
     * The frequency with which this Plan is called.
     */
    int _frequency;
    /**
     * The mapping from static to dynamic entry point.
     */
    mutable std::unordered_map<int64_t, std::unordered_set<int64_t>> _entryPointMap;
};

} // namespace alica
