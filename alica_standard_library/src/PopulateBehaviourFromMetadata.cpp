#include "PopulateBlackboardFromMetadata.h"
#include <yaml-cpp/yaml.h>

namespace utils
{

PopulateBlackboardFromMetadata::PopulateBlackboardFromMetadata(alica::BehaviourContext& context)
        : BasicBehaviour(context)
{
}

void PopulateBlackboardFromMetadata::initialiseParameters()
{
    readContext();
    if (auto error = verifyContext(); !error.empty()) {
        setError(error);
        return;
    }
    populateFromContext();
}

std::unique_ptr<PopulateBlackboardFromMetadata> PopulateBlackboardFromMetadata::create(alica::BehaviourContext& context)
{
    return std::make_unique<PopulateBlackboardFromMetadata>(context);
}

void PopulateBlackboardFromMetadata::readContext()
{
    alica::UnlockedBlackboard bb{*getBlackboard()};
    _context.metadataNode = YAML::Load(bb.get<std::string>("metadata"));
    _context.blackboard = bb.get<std::shared_ptr<alica::Blackboard>>("blackboard");
    _context.blueprint = bb.get<const alica::BlackboardBlueprint*>("blackboardBlueprint");
    _context.blackboardKeys = bb.get<std::vector<std::string>>("blackboardKeys");
}

std::string PopulateBlackboardFromMetadata::verifyContext()
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

void PopulateBlackboardFromMetadata::populateFromContext()
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

void PopulateBlackboardFromMetadata::setError(const std::string& error)
{
    alica::UnlockedBlackboard bb{*getBlackboard()};
    bb.set("error", error);
}

} // namespace utils
