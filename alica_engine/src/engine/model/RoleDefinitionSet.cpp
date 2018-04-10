/*
 * RoleDefinitionSet.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/RoleDefinitionSet.h"

namespace alica {

RoleDefinitionSet::RoleDefinitionSet() {}

RoleDefinitionSet::~RoleDefinitionSet() {}

void RoleDefinitionSet::setFileName(const std::string& fileName) {
    _fileName = fileName;
}

void RoleDefinitionSet::setRoles(const RoleVector& roles) {
    _roles = roles;
}
}  // namespace alica
