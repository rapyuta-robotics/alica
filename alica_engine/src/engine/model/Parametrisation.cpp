/*
 * Parametrisation.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Parametrisation.h"
#include "engine/model/Variable.h"
#include "engine/model/AbstractPlan.h"

namespace alica {

Parametrisation::Parametrisation() {
    this->subPlan = nullptr;
    this->subVar = nullptr;
    this->var = nullptr;
}

Parametrisation::~Parametrisation() {}

string Parametrisation::ToString() {
    stringstream ss;
    ss << "[Parametrisation: Var=" << this->var->getId();
    ss << " SubVar=" << this->subVar->getName() << " (" << this->subVar->getName() << "), ";
    ss << "SubPlan=" << this->subPlan->getName() << "]" << endl;
    return ss.str();
}

//================= Getter and Setter ========================

AbstractPlan* Parametrisation::getSubPlan() {
    return subPlan;
}

void Parametrisation::setSubPlan(AbstractPlan* subPlan) {
    this->subPlan = subPlan;
}

Variable* Parametrisation::getSubVar() {
    return subVar;
}

void Parametrisation::setSubVar(Variable* subVar) {
    this->subVar = subVar;
}

Variable* Parametrisation::getVar() {
    return var;
}

void Parametrisation::setVar(Variable* var) {
    this->var = var;
}

}  // namespace alica
