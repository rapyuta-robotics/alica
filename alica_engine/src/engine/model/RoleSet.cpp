/*
 * RoleSet.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */
#include <sstream>
#include "engine/model/RoleSet.h"

namespace alica {

RoleSet::RoleSet()
    : _usableWithPlanID(0)
    , _isDefault(false) {}

RoleSet::~RoleSet() {}

std::string RoleSet::toString() const {
    std::stringstream ss;
    ss << "#RoleSet: " << getName() << " " << getId() << std::endl;
    ss << "\t UsableWithPlanID: " << _usableWithPlanID << std::endl;
    ss << "\t Contains Mappings: " << _roleTaskMappings.size() << std::endl;
    for (const RoleTaskMapping* rtm : _roleTaskMappings) {
        ss << "\tRoleTaskMapping: " << rtm << std::endl;
    }
    ss << "#EndRoleSet" << std::endl;
    return ss.str();
}

void RoleSet::setIsDefault(bool isDefault) {
    _isDefault = isDefault;
}

void RoleSet::setRoleTaskMappings(const std::vector<RoleTaskMapping*>& roleTaskMappings) {
    _roleTaskMappings = roleTaskMappings;
}

void RoleSet::setUsableWithPlanId(int64_t usableWithPlanId) {
    _usableWithPlanID = usableWithPlanId;
}

}  // namespace alica
