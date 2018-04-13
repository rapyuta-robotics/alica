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

void CapabilityDefinitionSet::setCapabilities(const CapabilitySet& capabilities) {
    _capabilities = capabilities;
}

}  // namespace alica
