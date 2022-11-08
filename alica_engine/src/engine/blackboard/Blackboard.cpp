#include "engine/blackboard/Blackboard.h"

namespace alica
{

void BlackboardImpl::initDefaultValues()
{
    for (YAML::Node entry : node) {
    //     YAML::Node defaultValue = entry[Strings::defaultValue];
    //     if (defaultValue.Type() != YAML::NodeType::Null) {
    //         std::string key = entry[Strings::key].as<std::string>();
    //         std::string typeString = entry[Strings::stateType].as<std::string>();
    //         Converter::setValueFromYaml(key, typeString, defaultValue, *this);
    //     } else {
    //         // no default value, construct emtpy value
    //     }
    // }
    // Note: value has to be in pml
    // for (YAML::Node entry : node) {
    //     const auto& typeName = _pmlType.at(key);
    //     // get index of type in the variant
    //     for (std::size_t i = 0; i < BB_VALUE_TYPE_NAMES_SIZE; ++i) {
    //         if (BB_VALUE_TYPE_NAMES[i] == typeName) {
    //             // index = i + 1 (because of std::monostate)
    //             auto defValueIt = _pmlDefaultValues.find(key);
    //             if (defValueIt != _pmlDefaultValues.end()) {
    //                 // construct from default value
    //                 _values[key] = makeBBValueForIndex<true>::make(i + 1, defValueIt->second);
    //             } else {
    //                 // no default value, default construct
    //                 _values[key] = makeBBValueForIndex<false>::make(i + 1);
    //             }
    //         }
    //     }
    // }
    // Note: value has to be in pml
        const std::string& key = entry[Strings::key].as<std::string>();
        const auto& typeName = entry[Strings::stateType].as<std::string>();
        // get index of type in the variant
        for (std::size_t i = 0; i < BB_VALUE_TYPE_NAMES_SIZE; ++i) {
            if (BB_VALUE_TYPE_NAMES[i] == typeName) {
                // index = i + 1 (because of std::monostate)
                auto defaultValue = entry[Strings::defaultValue];
                // auto defValueIt = _pmlDefaultValues.find(key);
                if (!entry[Strings::defaultValue].IsNull()) {
                    std::cerr << "have default value: " << entry[Strings::defaultValue] << std::endl;
                    // construct from default value
                    // vals[key] = makeBBValueForIndex<true>::make(i + 1, defValueIt->second);
                    vals[key] = makeBBValueForIndex<true>::make(i + 1, entry[Strings::defaultValue].as<std::string>());
                } else {
                    // no default value, default construct
                    std::cerr << "no default value" << std::endl;
                    vals[key] = makeBBValueForIndex<false>::make(i + 1);
                    std::cerr << "finished make" << std::endl;
                }
            }
        }
        std::cerr << "finished initFromPML" << std::endl; 
    }
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
