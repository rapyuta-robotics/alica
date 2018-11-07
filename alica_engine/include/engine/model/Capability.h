/*
 * Capability.h
 *
 *  Created on: May 28, 2014
 *      Author: Paul Panin
 */

#ifndef CAPABILITY_H_
#define CAPABILITY_H_

#include "AlicaElement.h"
#include "engine/Types.h"

namespace alica
{
class CapValue;
class ModelFactory;

/**
 * A capability is used to match agents to roles.
 */
class Capability : public AlicaElement
{
public:
    Capability();
    virtual ~Capability();

    double similarityValue(const CapValue* roleVal, const CapValue* robotVal) const;

    const CapValueGrp& getCapValues() const { return _capValues; }

private:
    friend ModelFactory;
    /**
     * List of possible values for this capability
     */
    CapValueGrp _capValues;
};
} // namespace alica
#endif /* CAPABILITY_H_ */
