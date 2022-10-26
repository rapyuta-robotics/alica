#include "engine/modelmanagement/factories/BlackboardBlueprintFactory.h"

#include "engine/blackboard/BlackboardBlueprint.h"
#include "engine/modelmanagement/Strings.h"
#include <any>

namespace alica
{
std::unique_ptr<BlackboardBlueprint> BlackboardBlueprintFactory::create(const YAML::Node& node)
{

    auto blackboard = std::make_unique<BlackboardBlueprint>();
    blackboard->setBlackboardNode(node);
    for (const auto& entry : node) {
        auto key = getValue<std::string>(entry, Strings::key);
        blackboard->registerValue(key);
    }
    return blackboard;
}

std::unique_ptr<BlackboardBlueprint> BlackboardBlueprintFactory::createEmpty()
{
    return std::make_unique<BlackboardBlueprint>();
}

} // namespace alica
