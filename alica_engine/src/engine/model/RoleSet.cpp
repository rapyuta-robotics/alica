/*
 * RoleSet.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/RoleSet.h"

namespace alica {

RoleSet::RoleSet() {
    this->usableWithPlanID = 0;
    this->isDefault = false;
}

RoleSet::~RoleSet() {}

string RoleSet::toString() {
    stringstream ss;
    ss << "#RoleSet: " << this->name << " " << this->id << endl;
    ss << "\t UsableWithPlanID: " << this->usableWithPlanID << endl;
    ss << "\t Contains Mappings: " << this->roleTaskMappings.size() << endl;
    for (RoleTaskMapping* rtm : this->roleTaskMappings) {
        ss << "\tRoleTaskMapping: " << rtm << endl;
    }
    ss << "#EndRoleSet" << endl;
    return ss.str();
}

bool RoleSet::isIsDefault() const {
    return isDefault;
}

void RoleSet::setIsDefault(bool isDefault) {
    this->isDefault = isDefault;
}

list<RoleTaskMapping*>& RoleSet::getRoleTaskMappings() {
    return roleTaskMappings;
}

void RoleSet::setRoleTaskMappings(const list<RoleTaskMapping*> roleTaskMappings) {
    this->roleTaskMappings = roleTaskMappings;
}

long RoleSet::getUsableWithPlanId() const {
    return usableWithPlanID;
}

void RoleSet::setUsableWithPlanId(long usableWithPlanId) {
    usableWithPlanID = usableWithPlanId;
}

}  // namespace alica
