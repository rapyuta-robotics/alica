#pragma once

#include "AlicaElement.h"
#include <engine/RunningPlan.h>

#include <memory>

namespace alica
{

class State;
class Synchronisation;
class TransitionCondition;
class ModelFactory;
class TransitionFactory;
class ExpressionHandler;
class Blackboard;
class KeyMapping;

struct TransitionConditionContext
{
    const std::string name;
    const std::string libraryName;
    int64_t conditionConfId;
};

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
    const Synchronisation* getSynchronisation() const { return _synchronisation; }
    TransitionCondition* getTransitionCondition() const { return _transitionCondition; }
    const KeyMapping* getKeyMapping() const { return _keyMapping.get(); }

private:
    friend ModelFactory;
    friend TransitionFactory;
    friend ExpressionHandler;
    void setTransitionCondition(TransitionCondition* transitionCondition);
    void setInState(State* inState);
    void setOutState(State* outState);
    void setSynchronisation(Synchronisation* synchronisation);

    /**
     * The condition guarding this transition.
     */
    TransitionCondition* _transitionCondition;
    /**
     * The state from which this transition leads away.
     */
    const State* _inState;
    /**
     * The state this transition leads to
     */
    const State* _outState;
    /**
     * The Synchronisation this transition belongs to. Null if it does not belong to any.
     */
    const Synchronisation* _synchronisation;

    std::unique_ptr<KeyMapping> _keyMapping;
};

} // namespace alica
