#include "engine/modelmanagement/factories/BlackboardFactory.h"

#include "engine/blackboard/BlackboardBlueprint.h"
#include "engine/modelmanagement/Strings.h"
#include <any>

namespace alica
{
BlackboardBlueprint BlackboardFactory::create(const YAML::Node& node)
{

    BlackboardBlueprint blackboard;
    for (const auto& entry : node) {
        auto key = getValue<std::string>(entry, Strings::key);
        blackboard.registerValue(key);
    }
    return blackboard;
}

BlackboardBlueprint BlackboardFactory::createEmpty()
{
    return BlackboardBlueprint();
}

} // namespace alica
