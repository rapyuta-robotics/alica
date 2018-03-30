/*
 * CapabilityDefinitionSet.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/CapabilityDefinitionSet.h"

namespace alica {

CapabilityDefinitionSet::CapabilityDefinitionSet() {}

CapabilityDefinitionSet::~CapabilityDefinitionSet() {}

//================== Getter and Setter ===================================

list<Capability*>& CapabilityDefinitionSet::getCapabilities() {
    return capabilities;
}

void CapabilityDefinitionSet::setCapabilities(const list<Capability*>& capabilities) {
    this->capabilities = capabilities;
}

}  // namespace alica
