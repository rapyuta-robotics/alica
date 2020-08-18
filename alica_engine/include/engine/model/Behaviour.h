#pragma once

#include <list>
#include <string>

#include "AbstractPlan.h"
#include "engine/Types.h"

namespace alica
{

class AlicaTestSupportUtility;
class BasicBehaviour;
class ModelFactory;
class BehaviourFactory;
class PreCondition;
class RuntimeCondition;
class PostCondition;

/**
 * Represents a Behaviour within the plan tree
 */
class Behaviour : public AbstractPlan
{
public:
    Behaviour();
    virtual ~Behaviour();

    std::string toString(std::string indent = "") const;

    int getDeferring() const { return _deferring; }
    bool isEventDriven() const { return _eventDriven; }
    int getFrequency() const { return _frequency; }
    const RuntimeCondition* getRuntimeCondition() const { return _runtimeCondition; }
    const PreCondition* getPreCondition() const { return _preCondition; }
    const PostCondition* getPostCondition() const { return _postCondition; }

private:
    friend ModelFactory;
    friend BehaviourFactory;
    friend AlicaTestSupportUtility;

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
};

} // namespace alica
