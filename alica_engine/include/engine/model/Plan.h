#pragma once

#include "AbstractPlan.h"
#include "engine/Types.h"

namespace alica
{

class EntryPoint;
class FailureState;
class SuccessState;
class PostCondition;
class State;
class SyncTransition;
class Transition;
class ModelFactory;
class ExpressionHandler;
/**
 * An ALICA plan
 */
class Plan : public AbstractPlan
{
public:
    Plan(int64_t id);
    virtual ~Plan();

    const EntryPoint* getEntryPointTaskID(int64_t taskID) const;
    const EntryPoint* getEntryPointByID(int64_t epID) const;

    const EntryPointGrp& getEntryPoints() const { return _entryPoints; }

    const StateGrp& getStates() const { return _states; }
    const FailureStateGrp& getFailureStates() const { return _failureStates; }
    const SuccessStateGrp& getSuccessStates() const { return _successStates; }

    int getMaxCardinality() const { return _maxCardinality; }
    int getMinCardinality() const { return _minCardinality; }

    UtilityFunction* getUtilityFunction() const { return _utilityFunction.get(); }
    double getUtilityThreshold() const { return _utilityThreshold; }

    const PostCondition* getPostCondition() const { return _postCondition; }

    const TransitionGrp& getTransitions() const { return _transitions; }
    const SyncTransitionGrp& getSyncTransitions() const { return _syncTransitions; }

private:
    friend ModelFactory;
    friend ExpressionHandler; // TODO: get rid of this
    void setEntryPoints(const EntryPointGrp& entryPoints);
    void setFailureStates(const FailureStateGrp& failurePoints);
    void setSuccessStates(const SuccessStateGrp& succesPoints);
    void setMaxCardinality(int maxCardinality);
    void setMinCardinality(int minCardinality);
    void setPostCondition(const PostCondition* postCondition);
    void setStates(const StateGrp& states);
    void setSyncTransitions(const SyncTransitionGrp& syncTransitions);
    void setTransitions(const TransitionGrp& transitions);

    int _minCardinality;
    int _maxCardinality;
    EntryPointGrp _entryPoints;
    StateGrp _states;
    SuccessStateGrp _successStates;
    FailureStateGrp _failureStates;
    SyncTransitionGrp _syncTransitions;
    TransitionGrp _transitions;
    const PostCondition* _postCondition;
    /**
     * This plan's Utility function
     */
    // TOD: change shared to unique ptr (this requires a change to autogeneration templates)
    std::shared_ptr<UtilityFunction> _utilityFunction;
    /**
     * The utility threshold, the higher, the less likely dynamic changes are.
     */
    double _utilityThreshold;
};

} // namespace alica
