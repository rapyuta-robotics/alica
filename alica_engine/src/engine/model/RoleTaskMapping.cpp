/*
 * RoleTaskMapping.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/RoleTaskMapping.h"
#include "engine/model/Role.h"

namespace alica {

RoleTaskMapping::RoleTaskMapping() 
    : _role(nullptr)
{}

RoleTaskMapping::~RoleTaskMapping() {}

std::string RoleTaskMapping::toString() const {
    std::stringstream ss;
    ss << "#RoleTaskMapping: " << getName() << " " << getId() << std::endl;
    ss << "\t Role-Name: " << _role->getName() << std::endl;
    ss << "\t TaskPriorities: " << _taskPriorities.size() << std::endl;
    for (unordered_map<long, double>::const_iterator iterator = _taskPriorities.begin();
            iterator != _taskPriorities.end(); ++iterator) {
        const long l = iterator->first;
        const double val = iterator->second;
        ss << "\t" << l << " : " << val << std::endl;
    }
    ss << std::endl;
    ss << "#EndRoleTaskMapping" << std::endl;
    return ss.str();
}

void RoleTaskMapping::setRole(const Role* role) {
    _role = role;
}

void RoleTaskMapping::setTaskPriorities(const map<long, double>& taskPriorities) {
    _taskPriorities = taskPriorities;
}

}  // namespace alica
