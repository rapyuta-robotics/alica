#include <algorithm>
#include <alica_common_config/debug_output.h>
#include <any>
#include <assert.h>
#include <engine/blackboard/Blackboard.h>
#include <engine/blackboard/KeyMapping.h>
#include <iostream>
#include <sstream>

namespace alica
{

const KeyMapping::KeyMappingList& KeyMapping::getInputMapping() const
{
    return _inputMapping;
}

const KeyMapping::KeyMappingList& KeyMapping::getOutputMapping() const
{
    return _outputMapping;
}

void KeyMapping::addInputMapping(const std::string& parentKey, const std::string& childKey)
{
    assert(std::count(_inputMapping.begin(), _inputMapping.end(), std::make_pair(parentKey, childKey)) == 0);
    _inputMapping.push_back(std::make_pair(parentKey, childKey));
}

void KeyMapping::addOutputMapping(const std::string& parentKey, const std::string& childKey)
{
    assert(std::count(_outputMapping.begin(), _outputMapping.end(), std::make_pair(parentKey, childKey)) == 0);
    _outputMapping.push_back(std::make_pair(parentKey, childKey));
}

void KeyMapping::setInput(const Blackboard* parent_bb, Blackboard* child_bb, const KeyMapping* keyMapping) const
{
    const auto lockedParentBb = LockedBlackboardRO(*parent_bb);
    auto& childBb = child_bb->impl(); // Child not started yet, no other user exists, dont' use lock
    for (const auto& [parentKey, childKey] : keyMapping->getInputMapping()) {
        try {
            childBb.set(childKey, lockedParentBb.get(parentKey));
            ALICA_DEBUG_MSG("passing " << parentKey << " into " << childKey);
        } catch (std::exception& e) {
            ALICA_WARNING_MSG("Blackboard error passing " << parentKey << " into " << childKey << ". " << e.what());
        }
    }
}

void KeyMapping::setOutput(Blackboard* parent_bb, const Blackboard* child_bb, const KeyMapping* keyMapping) const
{
    auto lockedParentBb = LockedBlackboardRW(*parent_bb);
    const auto& childBb = child_bb->impl(); // Child is terminated, no other users exists, don't use lock
    for (const auto& [parentKey, childKey] : keyMapping->getOutputMapping()) {
        try {
            lockedParentBb.set(parentKey, childBb.get(childKey));
            ALICA_DEBUG_MSG("passing " << childKey << " into " << parentKey);
        } catch (std::exception& e) {
            ALICA_WARNING_MSG("Blackboard error passing " << childKey << " into " << parentKey << ". " << e.what());
        }
    }
}

} /* namespace alica */
