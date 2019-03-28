#include "engine/model/VariableBinding.h"

#include <sstream>

#include "engine/model/AbstractPlan.h"
#include "engine/model/Variable.h"

namespace alica
{

VariableBinding::VariableBinding()
        : _subPlan(nullptr)
        , _subVar(nullptr)
        , _var(nullptr)
{
}

VariableBinding::~VariableBinding() {}

std::string VariableBinding::toString() const
{
    std::stringstream ss;
    ss << "[VariableBinding: Var=" << _var->getId();
    ss << " SubVar=" << _subVar->getName() << " (" << _subVar->getId() << "), ";
    ss << "SubPlan=" << _subPlan->getName() << "]" << std::endl;
    return ss.str();
}

void VariableBinding::setSubPlan(const AbstractPlan* subPlan)
{
    _subPlan = subPlan;
}

void VariableBinding::setSubVar(const Variable* subVar)
{
    _subVar = subVar;
}

void VariableBinding::setVar(const Variable* var)
{
    _var = var;
}

} // namespace alica
