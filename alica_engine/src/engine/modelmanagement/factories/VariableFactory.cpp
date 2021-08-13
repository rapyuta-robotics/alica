#include "engine/modelmanagement/factories/VariableFactory.h"
#include "engine/model/Variable.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{
Variable* VariableFactory::create(const YAML::Node& variableNode)
{
    Variable* variable = new Variable();
    Factory::setAttributes(variableNode, variable);
    Factory::storeElement(variable, alica::Strings::variable);

    variable->_type = Factory::getValue<std::string>(variableNode, alica::Strings::variableType, alica::Strings::no_type);

    return variable;
}
} // namespace alica
