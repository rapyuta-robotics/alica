/*
 * Behaviour.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include <memory>

#include "engine/model/Behaviour.h"
#include "engine/model/BehaviourConfiguration.h"

namespace alica {

Behaviour::Behaviour() 
    : _implementation(nullptr)
{}


Behaviour::~Behaviour() {}

std::string Behaviour::toString() const {
    std::stringstream ss;
    ss << "#Behaviour: " << getName() << std::endl;
    ss << "\t Configurations: " << getConfigurations().size() << std::endl;
    for (const BehaviourConfiguration* bc : getConfigurations()) {
        ss << "\t" << bc->getName() << " " << bc->getId() << std::endl;
    }
    ss << "#EndBehaviour" << std::endl;
    return ss.str();
}


void Behaviour::setConfigurations(const BehaviourConfigurationSet& configurations) {
    _configurations = configurations;
}

void Behaviour::setFileName(const std::string& fileName) {
    _fileName = fileName;
}

void Behaviour::setImplementation(BasicBehaviour* implementation) {
    _implementation = implementation;
}
}  // namespace alica
