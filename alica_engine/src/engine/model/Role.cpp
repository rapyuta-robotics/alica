/*
 * Role.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Role.h"
#include "engine/model/RoleTaskMapping.h"
#include "engine/model/Characteristic.h"

namespace alica {

Role::Role() {
    this->roleDefinitionSet = nullptr;
    this->roleTaskMapping = nullptr;
}

Role::~Role() {}

double Role::getPriority(long taskId) {
    if (this->roleTaskMapping->getTaskPriorities().find(taskId) != this->roleTaskMapping->getTaskPriorities().end()) {
        return this->roleTaskMapping->getTaskPriorities().find(taskId)->second;
    } else {
        cerr << "ROLE DOES NOT HAVE A PRIORITY FOR TASK: " << taskId << endl;
        throw new exception;
    }
}

string Role::toString() {
    stringstream ss;
    ss << "#Role: " << this->name << " " << this->id << endl;
    ss << "\t Characteristics: " << this->characteristics.size() << endl;
    for (map<string, Characteristic*>::const_iterator iter = this->characteristics.begin();
            iter != this->characteristics.end(); iter++) {
        ss << "t" << iter->second->getName() << " : " << iter->second->getCapValue()->getName() << endl;
    }
    ss << endl;
    ss << "\tRTM TaskPriorities (" << this->roleTaskMapping->getId()
       << "): " << this->roleTaskMapping->getTaskPriorities().size() << endl;
    for (map<long, double>::const_iterator iterator = this->roleTaskMapping->getTaskPriorities().begin();
            iterator != this->roleTaskMapping->getTaskPriorities().end(); iterator++) {
        const long l = iterator->first;
        const double val = iterator->second;
        ss << "\t" << l << " : " << val << endl;
    }
    ss << endl;
    ss << "#EndRole" << endl;
    return ss.str();
}

//====================== Getter and Setter ==================

map<string, Characteristic*>& Role::getCharacteristics() {
    return characteristics;
}

const RoleDefinitionSet* Role::getRoleDefinitionSet() const {
    return roleDefinitionSet;
}

void Role::setRoleDefinitionSet(const RoleDefinitionSet* roleDefinitionSet) {
    this->roleDefinitionSet = roleDefinitionSet;
}

const RoleTaskMapping* Role::getRoleTaskMapping() const {
    return roleTaskMapping;
}

void Role::setRoleTaskMapping(RoleTaskMapping* roleTaskMapping) {
    this->roleTaskMapping = roleTaskMapping;
}
}  // namespace alica
