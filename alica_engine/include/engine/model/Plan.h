#pragma once

#include "AbstractPlan.h"
#include "engine/Types.h"

namespace alica
{

class BlackboardBlueprint;
class ConfigChangeListener;
class EntryPoint;
class ExpressionHandler;
class PlanFactory;
class PostCondition;
class PreCondition;
class RuntimeCondition;
class State;
class UtilityFunction;
/**
 * An ALICA plan
 */
class Plan : public AbstractPlan
{
public:
    Plan(ConfigChangeListener& configChangeListener, int64_t id);

    const EntryPoint* getEntryPointTaskID(int64_t taskID) const;
    const EntryPoint* getEntryPointByID(int64_t epID) const;
    const State* getStateByID(int64_t stateID) const;

    const EntryPointGrp& getEntryPoints() const { return _entryPoints; }

    const StateGrp& getStates() const { return _states; }
    const FailureStateGrp& getFailureStates() const { return _failureStates; }
    const SuccessStateGrp& getSuccessStates() const { return _successStates; }

    int getMaxCardinality() const { return _maxCardinality; }
    int getMinCardinality() const { return _minCardinality; }

    UtilityFunction* getUtilityFunction() const { return _utilityFunction.get(); }
    double getUtilityThreshold() const { return _utilityThreshold; }

    const TransitionGrp& getTransitions() const { return _transitions; }
    const SynchronisationGrp& getSynchronisations() const { return _synchronisations; }
    const RuntimeCondition* getRuntimeCondition() const { return _runtimeCondition; }
    const PreCondition* getPreCondition() const { return _preCondition; }

    int getFrequency() const { return _frequency; }

    const BlackboardBlueprint* getBlackboardBlueprint() const { return _blackboardBlueprint.get(); }

    std::string toString(std::string indent = "") const;

    AlicaTime getAuthorityTimeInterval() const { return _authorityTimeInterval; }
    void setAuthorityTimeInterval(AlicaTime authorityTimeInterval) const; // not a mistake, this is mutable

    void reload(const YAML::Node& config);

    std::string getLibraryName() const { return _libraryName; };
    std::string getImplementationName() const { return !_implementationName.empty() ? _implementationName : getName(); }

private:
    friend PlanFactory;
    friend ExpressionHandler; // TODO: get rid of this
    void setEntryPoints(const EntryPointGrp& entryPoints);
    void setFailureStates(const FailureStateGrp& failurePoints);
    void setSuccessStates(const SuccessStateGrp& succesPoints);
    void setMaxCardinality(int maxCardinality);
    void setMinCardinality(int minCardinality);
    void setPostCondition(const PostCondition* postCondition);
    void setRuntimeCondition(RuntimeCondition* runtimeCondition);
    void setPreCondition(PreCondition* preCondition);

    int _minCardinality;
    int _maxCardinality;
    EntryPointGrp _entryPoints;
    StateGrp _states;
    SuccessStateGrp _successStates;
    FailureStateGrp _failureStates;
    SynchronisationGrp _synchronisations;
    TransitionGrp _transitions;

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
     * If nullptr, it will simply receive a reference to its parents Blackboard
     * Otherwise, the mapped keys will be copied in and out of the plans Blackboard
     */
    std::unique_ptr<BlackboardBlueprint> _blackboardBlueprint;

    std::string _libraryName;
    std::string _implementationName; // Name of actual implementation file.  If empty, just use the plan name

    // TODO: move this to the authority module
    mutable AlicaTime _authorityTimeInterval;
};

} // namespace alica
