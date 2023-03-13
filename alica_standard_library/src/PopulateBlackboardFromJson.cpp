#include "PopulateBlackboardFromJson.h"
#include <yaml-cpp/yaml.h>

namespace alica_standard_library
{

PopulateBlackboardFromJson::PopulateBlackboardFromJson(alica::BehaviourContext& context)
        : BasicBehaviour(context)
{
}

void PopulateBlackboardFromJson::initialiseParameters()
{
    readContext();
    if (auto error = verifyContext(); !error.empty()) {
        setError(error);
        return;
    }
    populateFromContext();
    setSuccess();
}

std::unique_ptr<PopulateBlackboardFromJson> PopulateBlackboardFromJson::create(alica::BehaviourContext& context)
{
    return std::make_unique<PopulateBlackboardFromJson>(context);
}

void PopulateBlackboardFromJson::readContext()
{
    alica::UnlockedBlackboard bb{*getBlackboard()};
    _context.metadataNode = YAML::Load(bb.get<std::string>("metadata"));
    _context.blackboard = bb.get<std::shared_ptr<alica::Blackboard>>("blackboard");
    _context.blueprint = bb.get<const alica::BlackboardBlueprint*>("blackboardBlueprint");
    _context.blackboardKeys = bb.get<std::vector<std::string>>("blackboardKeys");
}

std::string PopulateBlackboardFromJson::verifyContext()
{
    if (!_context.blackboard) {
        return "Blackboard input error: key `blackboard` is null";
    }
    if (!_context.blueprint) {
        return "Blackboard input error: key `blueprint` is null";
    }
    for (const auto& key : _context.blackboardKeys) {
        auto keyInfoIt = std::find(_context.blueprint->begin(), _context.blueprint->end(), key);
        if (keyInfoIt == _context.blueprint->end()) {
            return "Blackboard input error: blueprint for key " + key + " not found";
        }
        auto typeIt = std::find(_knownTypes.begin(), _knownTypes.end(), keyInfoIt->second.type);
        if (typeIt == _knownTypes.end()) {
            return "Blackboard input error: type for key " + key + " is not supported, type: " + *typeIt;
        }
    }
    return std::string{};
}

void PopulateBlackboardFromJson::populateFromContext()
{
    for (const auto& key : _context.blackboardKeys) {
        auto keyInfoIt = std::find(_context.blueprint->begin(), _context.blueprint->end(), key);
        std::size_t typeIndex = 0;
        for (; typeIndex < _knownTypes.size(); ++typeIndex) {
            if (_knownTypes[typeIndex] == keyInfoIt->second.type) {
                break;
            }
        }
        setHelper(key, typeIndex, std::make_index_sequence<_knownTypes.size()>());
    }
}

void PopulateBlackboardFromJson::setError(const std::string& error)
{
    alica::UnlockedBlackboard bb{*getBlackboard()};
    bb.set("error", error);
    setFailure();
}

} // namespace alica_standard_library
