#include "engine/modelmanagement/factories/KeyMappingFactory.h"

#include "engine/modelmanagement/Strings.h"
#include <any>

namespace alica
{
KeyMapping KeyMappingFactory::create(const YAML::Node& node)
{

    KeyMapping keyMapping;
    if (Factory::isValid(node[alica::Strings::input])) {
        const auto& inputList = node[alica::Strings::input];
        for (const auto& entry : inputList) {
            keyMapping.addInputMapping(
                    Factory::getValue<std::string>(entry, alica::Strings::parent), Factory::getValue<std::string>(entry, alica::Strings::child));
        }
    }
    if (Factory::isValid(node[alica::Strings::output])) {
        const auto& outputList = node[alica::Strings::output];
        for (const auto& entry : outputList) {
            keyMapping.addOutputMapping(
                    Factory::getValue<std::string>(entry, alica::Strings::parent), Factory::getValue<std::string>(entry, alica::Strings::child));
        }
    }
    return keyMapping;
}

} // namespace alica
