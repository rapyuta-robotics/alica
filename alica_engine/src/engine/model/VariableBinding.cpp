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

std::string VariableBinding::toString(std::string indent) const
{
    std::stringstream ss;
    ss << indent << "[VariableBinding: Var=" << _var->getName() << " (" << _var->getId() << "),";
    ss << " SubVar=" << _subVar->getName() << " (" << _subVar->getId() << "),";
    ss << " SubPlan=" << _subPlan->getName() << "]" << std::endl;
    return ss.str();
}
} // namespace alica
