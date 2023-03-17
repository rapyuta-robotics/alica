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

void VariableBinding::setSubPlan(AbstractPlan* subPlan)
{
    if (_subPlan)
        std::cerr << "VariableBinding " << getName() << ": subPlan replaced" << std::endl;
    _subPlan = subPlan;
}

void VariableBinding::setVar(Variable* var)
{
    if (_var)
        std::cerr << "VariableBinding " << getName() << ": var replaced" << std::endl;
    _var = var;
}
void VariableBinding::setSubVar(Variable* subVar)
{
    if (_subVar)
        std::cerr << "VariableBinding " << getName() << ": subVar replaced" << std::endl;
    _subVar = subVar;
}

std::string VariableBinding::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "[VariableBinding: Var=" << _var->getName() << " (" << _var->getId() << "),";
    ss << " SubVar=" << _subVar->getName() << " (" << _subVar->getId() << "),";
    ss << " SubPlan=" << _subPlan->getName() << "]" << std::endl;
    return ss.str();
}
} // namespace alica
