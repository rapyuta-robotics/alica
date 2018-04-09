/*
 * Role.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Role.h"

#include "engine/model/RoleTaskMapping.h"
#include "engine/model/Characteristic.h"

#include <sstream>
#include <exception>
#include <iostream>


namespace alica {

Role::Role() 
    : _roleDefinitionSet(nullptr)
    , _roleTaskMapping(nullptr)
{}

Role::~Role() {}

double Role::getPriority(int64_t taskId) const {
    std::unordered_map<int64_t, double>::const_iterator it = _roleTaskMapping->getTaskPriorities().find(taskId);
    if (it != _roleTaskMapping->getTaskPriorities().end()) {
        return it->second;
    } else { //TODO move this check to start up
        std::cerr << "ROLE DOES NOT HAVE A PRIORITY FOR TASK: " << taskId << std::endl;
        throw new std::exception;
    }
}

std::string Role::toString() const {
    std::stringstream ss;
    ss << "#Role: " << getName() << " " << getId() << std::endl;
    ss << "\t Characteristics: " << _characteristics.size() << std::endl;
    for (std::unordered_map<std::string, const Characteristic*>::const_iterator iter = _characteristics.begin();
            iter != _characteristics.end(); ++iter) {
        ss << "t" << iter->second->getName() << " : " << iter->second->getCapValue()->getName() << std::endl;
    }
    ss << std::endl;
    ss << "\tRTM TaskPriorities (" << _roleTaskMapping->getId()
       << "): " << _roleTaskMapping->getTaskPriorities().size() << std::endl;
    for (std::unordered_map<long, double>::const_iterator iterator = _roleTaskMapping->getTaskPriorities().begin();
            iterator != _roleTaskMapping->getTaskPriorities().end(); ++iterator) {
        const long l = iterator->first;
        const double val = iterator->second;
        ss << "\t" << l << " : " << val << std::endl;
    }
    ss << std::endl;
    ss << "#EndRole" << std::endl;
    return ss.str();
}

//====================== Getter and Setter ==================


void Role::setRoleDefinitionSet(const RoleDefinitionSet* roleDefinitionSet) {
    _roleDefinitionSet = roleDefinitionSet;
}

void Role::setRoleTaskMapping(const RoleTaskMapping* roleTaskMapping) {
    _roleTaskMapping = roleTaskMapping;
}
}  // namespace alica
