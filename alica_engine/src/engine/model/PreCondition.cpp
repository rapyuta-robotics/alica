/*
 * PreCondition.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/PreCondition.h"
#include <sstream>
namespace alica {

PreCondition::PreCondition(int64_t id)
    : Condition(id)
    , _enabled(true)
{}

PreCondition::~PreCondition() {}

std::string PreCondition::toString() const {
    std::stringstream ss;
    ss << "#PreCondition: " << getName() << " " << getId() << (_enabled ? "enabled" : "disabled") << std::endl;
    ss << "\t ConditionString: " << getConditionString() << std::endl;
    ss << "#EndPreCondition" << std::endl;
    return ss.str();
}



void PreCondition::setEnabled(bool enabled) {
    _enabled = enabled;
}

}  // namespace alica
