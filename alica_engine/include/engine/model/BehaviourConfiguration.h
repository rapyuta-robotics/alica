/*
 * BehaviourConfiguration.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef BEHAVIOURCONFIGURATION_H_
#define BEHAVIOURCONFIGURATION_H_

#include "AbstractPlan.h"
#include "engine/Types.h"

namespace alica {
class Behaviour;
class ModelFactory;

/**
 * A Behaviour Configuration encapsulates a set of static parameters and a set of variables for a (Basic)Behaviour.
 *
 * The BehaviourConfiguration is indirectly derived from AlicaElement, therefore it
 * is owned by the PlanRepository and should never be deleted or changed by anybody else.
 */
class BehaviourConfiguration : public AbstractPlan {
public:
    BehaviourConfiguration();
    BehaviourConfiguration(int64_t id);
    virtual ~BehaviourConfiguration();

    std::string toString() const;

    int getDeferring() const { return _deferring; }
    bool isEventDriven() const { return _eventDriven; }
    int getFrequency() const { return _frequency; }

    const BehaviourParameterMap& getParameters() const { return _parameters; }
    const Behaviour* getBehaviour() const { return _behaviour; }

private:
    friend ModelFactory;
    void setDeferring(int deferring);
    void setEventDriven(bool eventDriven);
    void setFrequency(int frequency);
    void setParameters(const BehaviourParameterMap& parameters);
    void setBehaviour(const Behaviour* behaviour);
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
     * The set of static parameters of this Behaviour configuration. Usually parsed by
     * BasicBehaviour.InitializeParameters.
     */
    BehaviourParameterMap _parameters;
    /**
     * This configuration's Behaviour
     */
    const Behaviour* _behaviour;
};

}  // namespace alica

#endif /* BEHAVIOURCONFIGURATION_H_ */
