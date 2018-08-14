/*
 * Characteristic.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef CHARACTERISTIC_H_
#define CHARACTERISTIC_H_

#include <string.h>

#include "AlicaElement.h"
#include "CapValue.h"
#include "Capability.h"

namespace alica
{

class Capability;
class CapValue;
class ModelFactory;

/**
 * A characteristic encapsulates a Capability and aCapValue.
 */
class Characteristic : public AlicaElement
{
public:
    Characteristic();
    Characteristic(const Capability* cap, const CapValue* value);
    virtual ~Characteristic();

    std::string toString() const override;

    const Capability* getCapability() const { return _capability; }
    const CapValue* getCapValue() const { return _capValue; }
    double getWeight() const { return _weight; }

    void setCapability(const Capability* capability);
    void setWeight(double weight);
    void setCapValue(const CapValue* capValue);

private:
    friend ModelFactory;

    const Capability* _capability;
    const CapValue* _capValue;
    /**
     * The weight, used for role allocation.
     */
    double _weight;
};

} // namespace alica

#endif /* CHARACTERISTIC_H_ */
