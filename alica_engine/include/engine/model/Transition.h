#pragma once

#include "AlicaElement.h"
#include <engine/RunningPlan.h>

#include <memory>
namespace alica
{

class State;
class SyncTransition;
class PreCondition;
class ModelFactory;
class TransitionFactory;
class ExpressionHandler;

/**
 * Connects two States in a Plan
 */
class Transition : public AlicaElement
{
public:
    Transition();
    virtual ~Transition();

    const State* getOutState() const { return _outState; }
    const State* getInState() const { return _inState; }
    const SyncTransition* getSyncTransition() const { return _syncTransition; }
    const PreCondition* getPreCondition() const { return _preCondition; }

    bool evalCondition(const RunningPlan& r) const;

private:
    friend ModelFactory;
    friend TransitionFactory;
    friend ExpressionHandler;
    void setPreCondition(PreCondition* preCondition);
    void setInState(State* inState);
    void setOutState(State* outState);
    void setSyncTransition(SyncTransition* syncTransition);

    /**
     * The condition guarding this transition.
     */
    PreCondition* _preCondition;
    /**
     * The state from which this transition leads away.
     */
    const State* _inState;
    /**
     * The state this transition leads to
     */
    const State* _outState;
    /**
     * The SyncTransition this transition belongs to. Null if it does not belong to any.
     */
    const SyncTransition* _syncTransition;
};

} // namespace alica
