#include "engine/modelmanagement/factories/BlackboardBlueprintFactory.h"

#include "engine/blackboard/BlackboardBlueprint.h"
#include "engine/modelmanagement/Strings.h"
#include <any>

namespace alica
{
std::unique_ptr<BlackboardBlueprint> BlackboardBlueprintFactory::create(const YAML::Node& node)
{
    auto blueprint = std::make_unique<BlackboardBlueprint>();
    for (const auto& entry : node) {
        auto key = getValue<std::string>(entry, Strings::key);
        auto type = getValue<std::string>(entry, Strings::stateType);
        std::optional<std::string> defaultValue;
        if (!entry[Strings::defaultValue].IsNull()) {
            defaultValue = getValue<std::string>(entry, Strings::defaultValue);
        }
        blueprint->addKey(key, type, defaultValue);
    }
    return blueprint;
}

std::unique_ptr<BlackboardBlueprint> BlackboardBlueprintFactory::createEmpty()
{
    return std::make_unique<BlackboardBlueprint>();
}

} // namespace alica
