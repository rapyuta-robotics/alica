/*
 * CapabilityDefinitionSet.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef CAPABILITYDEFINITIONSET_H_
#define CAPABILITYDEFINITIONSET_H_

#include "AlicaElement.h"
#include "engine/Types.h"

namespace alica
{
class ModelFactory;
/**
 * A capability definition set holds all defined capabilities.
 */
class CapabilityDefinitionSet : public AlicaElement
{
public:
    CapabilityDefinitionSet();
    virtual ~CapabilityDefinitionSet();
    const CapabilityGrp& getCapabilities() const { return _capabilities; }

private:
    friend ModelFactory;
    void setCapabilities(const CapabilityGrp& capabilities);
    CapabilityGrp _capabilities;
};

} // namespace alica

#endif /* CAPABILITYDEFINITIONSET_H_ */
