#pragma once

#include <list>
#include <string>

#include "AbstractPlan.h"
#include "engine/Types.h"

namespace alica
{

class BehaviourConfiguration;
class BasicBehaviour;
class ModelFactory;

/**
 * Represents a Behaviour within the plan tree
 */
class Behaviour : public AbstractPlan
{
public:
    Behaviour();
    virtual ~Behaviour();

    std::string toString() const;

    const std::string& getFileName() const { return _fileName; }
    int getDeferring() const { return _deferring; }
    bool isEventDriven() const { return _eventDriven; }
    int getFrequency() const { return _frequency; }

private:
    friend ModelFactory;

    void setFileName(const std::string& fileName);
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
    std::string _fileName;
};

} // namespace alica
