#pragma once

#include <list>
#include <string>

#include "AbstractPlan.h"
#include "engine/Types.h"
#include "engine/blackboard/BlackboardBlueprint.h"

namespace alica
{
namespace test
{
class TestContext;
}

class BasicBehaviour;
class ModelFactory;
class BehaviourFactory;
class PreCondition;
class RuntimeCondition;
class PostCondition;
class AlicaEngine;

/**
 * Represents a Behaviour within the plan tree
 */
class Behaviour : public AbstractPlan
{
public:
    Behaviour(AlicaEngine* ae);
    virtual ~Behaviour();

    std::string toString(std::string indent = "") const;

    int getDeferring() const { return _deferring; }
    bool isEventDriven() const { return _eventDriven; }
    bool getInheritBlackboard() const { return _inheritBlackboard; }
    int getFrequency() const { return _frequency; }
    const RuntimeCondition* getRuntimeCondition() const { return _runtimeCondition; }
    const PreCondition* getPreCondition() const { return _preCondition; }
    const PostCondition* getPostCondition() const { return _postCondition; }
    BlackboardBlueprint getBlackboardBlueprint() const { return _blackboardBlueprint; }

private:
    friend ModelFactory;
    friend BehaviourFactory;
    friend alica::test::TestContext;

    void setDeferring(int deferring);
    void setEventDriven(bool eventDriven);
    void setFrequency(int frequency);
    /**
     * Specifies whether this Behaviour is run eventDriven. If it is not event driven, a timer will call it according to
     * Frequency and Deferring.
     */
    bool _eventDriven;
    /**
     * The frequency with which this Behaviour is called in case it is not EventDriven.
     */
    int _frequency;
    /**
     * The time in ms to wait before this Behaviour is executed for the first time after entering the corresponding
     * state. Has only effect for Behaviours not running in EventDriven mode.
     */
    int _deferring;
    /**
     * Specifies whether this behavior uses its parents' blackboard.
     * If so, it will simply receive a copy of its parents Blackboard
     * Otherwise, the mapped parameters will be copied in and out on init and termination respectively
     */
    bool _inheritBlackboard;
    /**
     * This behaviour's runtime condition.
     */
    RuntimeCondition* _runtimeCondition;
    /**
     * This behaviour's precondition
     */
    PreCondition* _preCondition;
    /**
     * This behaviour's postcondition
     */
    PostCondition* _postCondition;

    BlackboardBlueprint _blackboardBlueprint;
};

} // namespace alica
