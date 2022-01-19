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
    for (const auto& [parentKey, childKey] : inputMapping) {
        child_bb->impl().set(childKey, parent_bb->impl().vals.at(parentKey));
        std::cerr << "passing " << parentKey << " into " << childKey << " value " << parent_bb->impl().get<int>(parentKey) << std::endl;
    }
}

void KeyMapping::setOutput(Blackboard* parent_bb, const Blackboard* child_bb) const
{
    for (const auto& [parentKey, childKey] : outputMapping) {
        parent_bb->impl().set(parentKey, child_bb->impl().vals.at(childKey));
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
