#include "PopulateBlackboard.h"
#include <yaml-cpp/yaml.h>

namespace alica_standard_library
{

PopulateBlackboard::PopulateBlackboard(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PopulateBlackboard::onInit()
{
    alica::UnlockedBlackboard bb{*getBlackboard()};
    _json = YAML::Load(bb.get<std::string>("data"));

    for (const auto& [key, keyInfo] : *getBlackboardBlueprint()) {
        if (keyInfo.access == "protected") {
            if (!_json[key]) {
                alica::Logging::logDebug("PopulateBlackboard") << key << " not present in `data`, skipping";
                continue;
            } else if (_json[key].IsNull()) {
                alica::Logging::logDebug("PopulateBlackboard") << key << " is null in `data`, skipping";
                continue;
            }
            set(key, keyInfo.type);
        }
    }
}

std::unique_ptr<PopulateBlackboard> PopulateBlackboard::create(alica::PlanContext& context)
{
    return std::make_unique<PopulateBlackboard>(context);
}

void PopulateBlackboard::set(const std::string& key, const std::string& type)
{
    auto it = std::find(_knownTypes.begin(), _knownTypes.end(), type);
    if (it != _knownTypes.end()) {
        setHelper(key, it - _knownTypes.begin(), std::make_index_sequence<_knownTypes.size()>());
    }
}

} // namespace alica_standard_library
