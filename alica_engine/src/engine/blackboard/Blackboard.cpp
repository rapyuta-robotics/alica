#include "engine/blackboard/Blackboard.h"

namespace alica
{

void BlackboardImpl::initDefaultValues()
{
    for (YAML::Node entry : node) {
        YAML::Node defaultValue = entry[Strings::defaultValue];
        if (defaultValue.Type() != YAML::NodeType::Null) {
            std::string key = entry[Strings::key].as<std::string>();
            std::string typeString = entry[Strings::stateType].as<std::string>();
            Converter::setDefaultValue(key, typeString, defaultValue, *this);
        }
    }
}


void Converter::setDefaultValue(const std::string& key, const std::string& typeName, const YAML::Node& defaultValue, BlackboardImpl& bb)
{
    if (typeName == "std::any") {
        bb.set<std::any>(key, defaultValue.as<std::string>());
        return;
    }
    static constexpr std::size_t numTypes = sizeof(typeNamesYAML) / sizeof(const char*);
    for (std::size_t i = 0; i < numTypes; ++i) {
        if (typeName == typeNamesYAML[i]) {
            auto value = makeAnyFromIndex(TypesYAML{}, std::make_index_sequence<numTypes>{}, i, defaultValue);
            // store value on the blackboard
            setBlackboardValue(TypesYAML{}, std::make_index_sequence<numTypes>{}, i, key, value, bb);
            break;
        }
    }
}
} // namespace alica
