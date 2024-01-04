#include "engine/modelmanagement/factories/PlaceholderFactory.h"

#include "engine/model/Placeholder.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/BlackboardBlueprintFactory.h"

namespace alica
{
Placeholder* PlaceholderFactory::create(const YAML::Node& node)
{
    Placeholder* placeholder = new Placeholder();
    Factory::setAttributes(node, placeholder);
    Factory::storeElement(placeholder, alica::Strings::placeholder);

    if (Factory::isValid(node[alica::Strings::blackboard])) {
        placeholder->_blackboardBlueprint = BlackboardBlueprintFactory::create(node[alica::Strings::blackboard]);
    } else {
        placeholder->_blackboardBlueprint = BlackboardBlueprintFactory::createEmpty();
    }
    return placeholder;
}
} // namespace alica
