/*
 * BehaviourConfiguration.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/BehaviourConfiguration.h"
#include "engine/model/Behaviour.h"

namespace alica {

BehaviourConfiguration::BehaviourConfiguration() 
    : _eventDriven(false)
    , _frequency(30)
    , _deferring(0)
    , _behaviour(nullptr) {}}

BehaviourConfiguration::BehaviourConfiguration(int64_t id)
    : AlicaElement(id)
    , _eventDriven(false)
    , _frequency(30)
    , _deferring(0)
    , _behaviour(nullptr) {}

BehaviourConfiguration::~BehaviourConfiguration() {}

std::string BehaviourConfiguration::toString() const {
    std::stringstream ss;
    ss << "#BehaviourConfiguration: " << getName() << " " << getId() << endl;
    ss << "\t Behaviour: ";
    if (getBehaviour() != NULL) {
        ss << name << " " << getBehaviour()->getId();
    }
    ss << std::endl;
    ss << "\t Deferring: " + to_string(getDeferring()) << std::endl;
    ss << "\t Frequency: " + to_string(getFrequency()) << std::endl;
    ss << "\t MasterPlan?: " + to_string(isMasterPlan()) << std::endl;
    ss << "\t Parameters: " + to_string(getParameters()->size()) << std::endl;

    if (getParameters()->size() != 0) {
        for (BehaviourParameterMap::const_iterator iter = getParameters()->begin();
                iter != getParameters()->end(); iter++) {
            const std::string& s = iter->first;
            const std::string& val = iter->second;
            ss << "\t" + s << " : " << val << std::endl;
        }
    }
    ss << std::endl;
    ss << "#EndBehaviourConfiguration" << std::endl;

    return ss.str();
}

void BehaviourConfiguration::setDeferring(int deferring) {
    _deferring = deferring;
}

void BehaviourConfiguration::setEventDriven(bool eventDriven) {
    _eventDriven = eventDriven;
}

void BehaviourConfiguration::setFrequency(int frequency) {
    _frequency = frequency;
}

void BehaviourConfiguration::setParameters(shared_ptr<map<string, string>> parameters) {
    _parameters = parameters;
}


void BehaviourConfiguration::setBehaviour(const Behaviour* behaviour) {
    _behaviour = behaviour;
}

}  // namespace alica
