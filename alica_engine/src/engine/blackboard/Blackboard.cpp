#include "engine/blackboard/Blackboard.h"

namespace alica
{
void Converter::setDefaultValue(const std::string& key, const std::string& typeName, const YAML::Node& defaultValue, BlackboardImpl& bb)
{
    if (typeName == "std::any") {
        bb.set<std::any>(key, defaultValue.as<std::string>());
        return;
    }
    static constexpr std::size_t numTypes = sizeof(typeNames) / sizeof(const char*);
    for (std::size_t i = 0; i < numTypes; ++i) {
        if (typeName == typeNames[i]) {
            auto value = makeAnyFromIndex(Types{}, std::make_index_sequence<numTypes>{}, i, defaultValue);
            // store value on the blackboard
            // get correct type, pass to set
            // bb.set<>(key, value);
            break;
        }
    }
}
} // namespace alica
