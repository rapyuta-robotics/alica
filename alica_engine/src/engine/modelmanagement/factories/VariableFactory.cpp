#include "engine/modelmanagement/factories/VariableFactory.h"
#include "engine/model/Variable.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{
Variable* VariableFactory::create(const YAML::Node& variableNode)
{
    auto* variable = new Variable(Factory::getValue<std::string>(variableNode, alica::Strings::variableType, alica::Strings::no_type));
    Factory::setAttributes(variableNode, variable);
    Factory::storeElement(variable, alica::Strings::variable);

    return variable;
}

Variable* VariableFactory::create(int64_t id, const std::string& name, const std::string& type)
{
    auto* variable = new Variable(id, name, type);
    Factory::storeElement(variable, alica::Strings::variable);

    return variable;
}
} // namespace alica
