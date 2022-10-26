#include "engine/blackboard/Blackboard.h"

namespace alica
{

void BlackboardImpl::initDefaultValues()
{
    // for (YAML::Node entry : node) {
    //     YAML::Node defaultValue = entry[Strings::defaultValue];
    //     if (defaultValue.Type() != YAML::NodeType::Null) {
    //         std::string key = entry[Strings::key].as<std::string>();
    //         std::string typeString = entry[Strings::stateType].as<std::string>();
    //         Converter::setValueFromYaml(key, typeString, defaultValue, *this);
    //     }
    // }
}

void Converter::setValueFromYaml(const std::string& key, const std::string& typeName, const YAML::Node& valueNode, BlackboardImpl& bb)
{
    if (typeName == "std::any") {
        bb.set<std::any>(key, valueNode.as<std::string>());
        return;
    }
    static constexpr std::size_t numTypes = sizeof(typeNamesYAML) / sizeof(const char*);
    for (std::size_t i = 0; i < numTypes; ++i) {
        if (typeName == typeNamesYAML[i]) {
            auto value = makeVariantFromIndex(TypesYAML{}, std::make_index_sequence<numTypes>{}, i, valueNode);
            // store value on the blackboard
            bb.vals.at(key) = value;
            return;
        }
    }
}

void Converter::setValue(const std::string& key, const BlackboardValueType& value, const std::string& typeName, BlackboardImpl& bb)
{
    if (typeName == "std::any") {
        bb.set<std::any>(key, std::get<std::any>(value));
        return;
    }

    static constexpr std::size_t numTypes = sizeof(typeNamesYAML) / sizeof(const char*);
    for (std::size_t i = 0; i < numTypes; ++i) {
        if (typeName == typeNamesYAML[i]) {
            if (i == value.index()) {
                // variant is of type typeName
                bb.vals.at(key) = value;
                return;
            }
            // variant is not of type typeName
            throw BlackboardTypeMismatch(typeName, typeNamesYAML[value.index()]);
        }
    }
}
} // namespace alica
