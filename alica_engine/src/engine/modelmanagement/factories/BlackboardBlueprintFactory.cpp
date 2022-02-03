#include "engine/modelmanagement/factories/BlackboardBlueprintFactory.h"

#include "engine/blackboard/BlackboardBlueprint.h"
#include "engine/modelmanagement/Strings.h"
#include <any>

namespace alica
{
const BlackboardBlueprint* BlackboardBlueprintFactory::create(const YAML::Node& node)
{

    auto blackboard = new BlackboardBlueprint();
    for (const auto& entry : node) {
        auto key = getValue<std::string>(entry, Strings::key);
        blackboard->registerValue(key);
    }
    return blackboard;
}

const BlackboardBlueprint* BlackboardBlueprintFactory::createEmpty()
{
    return new BlackboardBlueprint();
}

} // namespace alica
