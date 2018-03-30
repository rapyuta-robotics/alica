/*
 * BehaviourConfiguration.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/Behaviour.h"

namespace alica {

BehaviourConfiguration::BehaviourConfiguration() {
    this->eventDriven = false;
    this->frequency = 30;
    this->deferring = 0;
    this->behaviour = nullptr;
    parameters = make_shared<map<string, string>>();
}

BehaviourConfiguration::BehaviourConfiguration(long id) : BehaviourConfiguration() {
    this->id = id;
}

BehaviourConfiguration::~BehaviourConfiguration() {}

string BehaviourConfiguration::toString() {
    stringstream ss;
    ss << "#BehaviourConfiguration: " << this->getName() << " " << this->getId() << endl;
    ss << "\t Behaviour: ";
    if (this->getBehaviour() != NULL) {
        ss << name << " " << this->getBehaviour()->getId();
    }
    ss << endl;
    ss << "\t Deferring: " + to_string(this->getDeferring()) << endl;
    ss << "\t Frequency: " + to_string(this->getFrequency()) << endl;
    ss << "\t MasterPlan?: " + to_string(this->isMasterPlan()) << endl;
    ss << "\t Parameters: " + to_string(this->getParameters()->size()) << endl;

    if (this->getParameters()->size() != 0) {
        for (map<string, string>::const_iterator iter = this->getParameters()->begin();
                iter != this->getParameters()->end(); iter++) {
            const string s = iter->first;
            const string val = iter->second;
            ss << "\t" + s << " : " << val << endl;
        }
    }
    ss << endl;
    ss << "#EndBehaviourConfiguration" << endl;

    return ss.str();
}

int BehaviourConfiguration::getDeferring() const {
    return deferring;
}

void BehaviourConfiguration::setDeferring(int deferring) {
    this->deferring = deferring;
}

bool BehaviourConfiguration::isEventDriven() const {
    return eventDriven;
}

void BehaviourConfiguration::setEventDriven(bool eventDriven) {
    this->eventDriven = eventDriven;
}

int BehaviourConfiguration::getFrequency() const {
    return frequency;
}

void BehaviourConfiguration::setFrequency(int frequency) {
    this->frequency = frequency;
}

shared_ptr<map<string, string>> BehaviourConfiguration::getParameters() {
    return parameters;
}

void BehaviourConfiguration::setParameters(shared_ptr<map<string, string>> parameters) {
    this->parameters = parameters;
}

const Behaviour* BehaviourConfiguration::getBehaviour() const {
    return behaviour;
}

void BehaviourConfiguration::setBehaviour(const Behaviour* behaviour) {
    this->behaviour = behaviour;
}

}  // namespace alica
