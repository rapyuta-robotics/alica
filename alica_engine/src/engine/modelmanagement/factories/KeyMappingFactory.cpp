#include "engine/modelmanagement/factories/KeyMappingFactory.h"

#include "engine/modelmanagement/Strings.h"

namespace alica
{
std::unique_ptr<KeyMapping> KeyMappingFactory::create(const YAML::Node& node)
{
    auto keyMapping = std::make_unique<KeyMapping>();
    if (Factory::isValid(node[alica::Strings::input])) {
        const auto& inputList = node[alica::Strings::input];
        for (const auto& entry : inputList) {
            if (Factory::isValid(entry[alica::Strings::parentKey])) {
                keyMapping->addInputMapping(
                        Factory::getValue<std::string>(entry, alica::Strings::parentKey), Factory::getValue<std::string>(entry, alica::Strings::childKey));
            } else {
                keyMapping->addInputValueMapping(
                        Factory::getValue<std::string>(entry, alica::Strings::value), Factory::getValue<std::string>(entry, alica::Strings::childKey));
            }
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
