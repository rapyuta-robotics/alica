/*
 * BehaviourConfiguration.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef BEHAVIOURCONFIGURATION_H_
#define BEHAVIOURCONFIGURATION_H_

#include <map>
#include <string>

#include "AbstractPlan.h"

using namespace std;
namespace alica {
class Behaviour;

/**
 * A Behaviour Configuration encapsulates a set of static parameters and a set of variables for a (Basic)Behaviour.
 *
 * The BehaviourConfiguration is indirectly derived from AlicaElement, therefore it
 * is owned by the PlanRepository and should never be deleted or changed by anybody else.
 */
class BehaviourConfiguration : public AbstractPlan {
public:
    BehaviourConfiguration();
    BehaviourConfiguration(long id);
    virtual ~BehaviourConfiguration();

    string toString();

    int getDeferring() const;
    void setDeferring(int deferring);
    bool isEventDriven() const;
    void setEventDriven(bool eventDriven);
    int getFrequency() const;
    void setFrequency(int frequency);
    shared_ptr<map<string, string>> getParameters();
    void setParameters(const shared_ptr<map<string, string>> parameters);
    const Behaviour* getBehaviour() const;
    void setBehaviour(const Behaviour* behaviour);

private:
    /**
     * Specifies whether this Behaviour is run eventDriven. If it is not event driven, a timer will call it according to
     * Frequency and Deferring.
     */
    bool eventDriven;
    /**
     * The frequency with which this Behaviour is called in case it is not EventDriven.
     */
    int frequency;
    /**
     * The time in ms to wait before this Behaviour is executed for the first time after entering the corresponding
     * state. Has only effect for Behaviours not running in EventDriven mode.
     */
    int deferring;
    /**
     * The set of static parameters of this Behaviour configuration. Usually parsed by
     * BasicBehaviour.InitializeParameters.
     */
    shared_ptr<map<string, string>> parameters;
    /**
     * This configuration's Behaviour
     */
    const Behaviour* behaviour;
};

}  // namespace alica

#endif /* BEHAVIOURCONFIGURATION_H_ */
