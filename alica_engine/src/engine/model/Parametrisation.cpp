/*
 * Parametrisation.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Parametrisation.h"

#include <sstream>

#include "engine/model/Variable.h"
#include "engine/model/AbstractPlan.h"

namespace alica {

Parametrisation::Parametrisation() 
    : _subPlan(nullptr)
    , _subVar(nullptr)
    , _var(nullptr) {}

Parametrisation::~Parametrisation() {}

std::string Parametrisation::ToString() {
    std::stringstream ss;
    ss << "[Parametrisation: Var=" << _var->getId();
    ss << " SubVar=" << _subVar->getName() << " (" << _subVar->getName() << "), ";
    ss << "SubPlan=" << _subPlan->getName() << "]" << std::endl;
    return ss.str();
}

void Parametrisation::setSubPlan(AbstractPlan* subPlan) {
    _subPlan = subPlan;
}

void Parametrisation::setSubVar(Variable* subVar) {
    _subVar = subVar;
}

void Parametrisation::setVar(Variable* var) {
    _var = var;
}

}  // namespace alica
