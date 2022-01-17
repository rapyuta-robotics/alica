#include "engine/modelmanagement/factories/BlackboardFactory.h"

#include "engine/blackboard/Blackboard.h"
#include "engine/modelmanagement/Strings.h"
#include <any>

namespace alica
{
std::unique_ptr<Blackboard> BlackboardFactory::create(const YAML::Node& node)
{

    auto blackboard = std::make_unique<Blackboard>();
    auto& blackboardInternal = blackboard->impl();
    for (const auto& entry : node) {
        auto key = getValue<std::string>(entry, Strings::key);
        blackboardInternal.registerValue(key);
    }
    return blackboard;
}

} // namespace alica
