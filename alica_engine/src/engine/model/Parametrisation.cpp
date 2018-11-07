/*
 * Parametrisation.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/Parametrisation.h"

#include <sstream>

#include "engine/model/AbstractPlan.h"
#include "engine/model/Variable.h"

namespace alica
{

Parametrisation::Parametrisation()
        : _subPlan(nullptr)
        , _subVar(nullptr)
        , _var(nullptr)
{
}

Parametrisation::~Parametrisation() {}

std::string Parametrisation::toString() const
{
    std::stringstream ss;
    ss << "[Parametrisation: Var=" << _var->getId();
    ss << " SubVar=" << _subVar->getName() << " (" << _subVar->getId() << "), ";
    ss << "SubPlan=" << _subPlan->getName() << "]" << std::endl;
    return ss.str();
}

void Parametrisation::setSubPlan(const AbstractPlan* subPlan)
{
    _subPlan = subPlan;
}

void Parametrisation::setSubVar(const Variable* subVar)
{
    _subVar = subVar;
}

void Parametrisation::setVar(const Variable* var)
{
    _var = var;
}

} // namespace alica
