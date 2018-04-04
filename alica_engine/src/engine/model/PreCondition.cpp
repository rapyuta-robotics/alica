/*
 * PreCondition.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/PreCondition.h"

namespace alica {

PreCondition::PreCondition(int64_t id)
    : Condition(id)
    , _enabled(true)
{}

PreCondition::~PreCondition() {}

std::string PreCondition::toString() const {
    stringstream ss;
    ss << "#PreCondition: " << getName() << " " << getId() << (_enabled ? "enabled" : "disabled") << endl;
    ss << "\t ConditionString: " << getConditionString() << endl;
    ss << "#EndPreCondition" << endl;
    return ss.str();
}



void PreCondition::setEnabled(bool enabled) {
    _enabled = enabled;
}

}  // namespace alica
