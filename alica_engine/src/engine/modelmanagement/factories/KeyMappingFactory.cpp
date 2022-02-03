#include "engine/modelmanagement/factories/KeyMappingFactory.h"

#include "engine/modelmanagement/Strings.h"
#include <any>

namespace alica
{
const KeyMapping* KeyMappingFactory::create(const YAML::Node& node)
{

    auto keyMapping = new KeyMapping();
    if (Factory::isValid(node[alica::Strings::input])) {
        const auto& inputList = node[alica::Strings::input];
        for (const auto& entry : inputList) {
            keyMapping->addInputMapping(
                    Factory::getValue<std::string>(entry, alica::Strings::parentKey), Factory::getValue<std::string>(entry, alica::Strings::childKey));
        }
    }
    if (Factory::isValid(node[alica::Strings::output])) {
        const auto& outputList = node[alica::Strings::output];
        for (const auto& entry : outputList) {
            keyMapping->addOutputMapping(
                    Factory::getValue<std::string>(entry, alica::Strings::parentKey), Factory::getValue<std::string>(entry, alica::Strings::childKey));
        }
    }
    return keyMapping;
}

} // namespace alica
