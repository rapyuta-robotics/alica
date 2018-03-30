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

Behaviour::Behaviour() {
    this->implementation = nullptr;
}

Behaviour::Behaviour(string name) : Behaviour() {
    this->name = name;
}

Behaviour::~Behaviour() {}

string Behaviour::toString() {
    stringstream ss;
    ss << "#Behaviour: " << this->getName() << endl;
    ss << "\t Configurations: " << this->getConfigurations().size() << endl;
    for (BehaviourConfiguration* bc : this->getConfigurations()) {
        ss << "\t" << bc->getName() << " " << bc->getId() << endl;
    }
    ss << "#EndBehaviour" << endl;
    return ss.str();
}

list<BehaviourConfiguration*>& Behaviour::getConfigurations() {
    return configurations;
}

void Behaviour::setConfigurations(const list<BehaviourConfiguration*>& configurations) {
    this->configurations = configurations;
}

const string& Behaviour::getFileName() const {
    if (this->getFileName().empty()) {
        static string result = name + ".beh";
        return result;
    }
    return fileName;
}

void Behaviour::setFileName(const string& fileName) {
    this->fileName = fileName;
}

BasicBehaviour* alica::Behaviour::getImplementation() {
    return implementation;
}

void alica::Behaviour::setImplementation(BasicBehaviour* implementation) {
    this->implementation = implementation;
}
}  // namespace alica
