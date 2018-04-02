/*
 * CapabilityDefinitionSet.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef CAPABILITYDEFINITIONSET_H_
#define CAPABILITYDEFINITIONSET_H_

#include <list>

#include "AlicaElement.h"

using namespace std;
namespace alica {

class Capability;

/**
 * A capability definition set holds all defined capabilities.
 */
class CapabilityDefinitionSet : public AlicaElement {
public:
    CapabilityDefinitionSet();
    virtual ~CapabilityDefinitionSet();
    list<Capability*>& getCapabilities();

private:
    list<Capability*> capabilities;
    void setCapabilities(const list<Capability*>& capabilities);
};

}  // namespace alica

#endif /* CAPABILITYDEFINITIONSET_H_ */
