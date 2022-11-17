#include "engine/blackboard/Blackboard.h"

namespace alica
{

void BlackboardImpl::initDefaultValues()
{
    for (YAML::Node entry : node) {
        // Note: value has to be in pml
        const std::string& key = entry[Strings::key].as<std::string>();
        const auto& typeName = entry[Strings::stateType].as<std::string>();
        // get index of type in the variant
        for (std::size_t i = 0; i < BB_VALUE_TYPE_NAMES_SIZE; ++i) {
            if (BB_VALUE_TYPE_NAMES[i] == typeName) {
                // index = i + 1 (because of std::monostate)
                auto defaultValue = entry[Strings::defaultValue];
                if (!entry[Strings::defaultValue].IsNull()) {
                    // construct from default value
                    _vals[key] = makeBBValueForIndex<true>::make(i + 1, entry[Strings::defaultValue].as<std::string>());
                } else {
                    // no default value, default construct
                    _vals[key] = makeBBValueForIndex<false>::make(i + 1);
                }
            }
        }
    }
}
} // namespace alica
