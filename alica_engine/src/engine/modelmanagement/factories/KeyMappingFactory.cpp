#include "engine/modelmanagement/factories/KeyMappingFactory.h"

#include "engine/modelmanagement/Strings.h"
#include <any>

namespace alica
{
std::unique_ptr<KeyMapping> KeyMappingFactory::create(const YAML::Node& node)
{

    auto keyMapping = std::make_unique<KeyMapping>();
    if (Factory::isValid(node[alica::Strings::input])) {
        const auto& inputList = node[alica::Strings::input];
        for (const auto& entry : inputList) {
            // TODO: Remove ifelse once backend has been updated and all keymapping nodes have an (empty) value field
            if ((Factory::isValid(entry[alica::Strings::value]))) {
                keyMapping->addInputMapping(Factory::getValue<std::string>(entry, alica::Strings::parentKey),
                        Factory::getValue<std::string>(entry, alica::Strings::childKey), Factory::getValue<std::string>(entry, alica::Strings::value));
            } else {
                keyMapping->addInputMapping(
                        Factory::getValue<std::string>(entry, alica::Strings::parentKey), Factory::getValue<std::string>(entry, alica::Strings::childKey));
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
