#include <algorithm>
#include <any>
#include <assert.h>
#include <engine/blackboard/Blackboard.h>
#include <engine/blackboard/KeyMapping.h>
#include <iostream>

namespace alica
{

void KeyMapping::setInput(const Blackboard* parent_bb, Blackboard* child_bb) const
{
    const auto lockedParentBb = LockedBlackboardRO(*parent_bb);
    auto lockedChildBb = LockedBlackboardRW(*child_bb);
    for (const auto& [parentKey, childKey] : inputMapping) {
        lockedChildBb.set(childKey, lockedParentBb.get(parentKey));
        std::cerr << "passing " << parentKey << " into " << childKey << std::endl;
    }
}

void KeyMapping::setOutput(Blackboard* parent_bb, const Blackboard* child_bb) const
{
    auto lockedParentBb = LockedBlackboardRW(*parent_bb);
    const auto lockedChildBb = LockedBlackboardRO(*child_bb);
    for (const auto& [parentKey, childKey] : outputMapping) {
        lockedParentBb.set(parentKey, lockedChildBb.get(childKey));
        std::cerr << "passing " << childKey << " into " << parentKey << std::endl;
    }
}

void KeyMapping::addInputMapping(const std::string& parentKey, const std::string& childKey)
{
    assert(std::count(inputMapping.begin(), inputMapping.end(), std::make_pair(parentKey, childKey)) == 0);
    inputMapping.push_back(std::make_pair(parentKey, childKey));
}

void KeyMapping::addOutputMapping(const std::string& parentKey, const std::string& childKey)
{
    assert(std::count(outputMapping.begin(), outputMapping.end(), std::make_pair(parentKey, childKey)) == 0);
    outputMapping.push_back(std::make_pair(parentKey, childKey));
}

} /* namespace alica */
