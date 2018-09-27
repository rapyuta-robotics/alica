/*
 * Characteristic.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Characteristic.h"
#include <sstream>

namespace alica
{

Characteristic::Characteristic()
        : _capability(nullptr)
        , _capValue(nullptr)
        , _weight(0.0)
{
}

Characteristic::Characteristic(const Capability* cap, const CapValue* value)
        : _capability(cap)
        , _capValue(value)
        , _weight(0.0)
{
}

Characteristic::~Characteristic() {}

std::string Characteristic::toString() const
{
    std::stringstream ss;
    ss << "#Characteristic " << std::endl;
    ss << "\t Capability: " << _capability->getName() << std::endl;
    ss << "\t CapValue: " << _capValue->getName() << std::endl;
    ss << "\t Weight: " << _weight << std::endl;
    return ss.str();
}

void Characteristic::setCapability(const Capability* capability)
{
    _capability = capability;
}

void Characteristic::setCapValue(const CapValue* capValue)
{
    _capValue = capValue;
}

void Characteristic::setWeight(double weight)
{
    _weight = weight;
}

} // namespace alica
