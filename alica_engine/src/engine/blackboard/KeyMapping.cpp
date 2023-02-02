#include <algorithm>
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

void KeyMapping::addInputMapping(const std::variant<std::string, std::string>& parentKeyOrConstant, const std::string& childKey)
{
    assert(std::count_if(_inputMapping.begin(), _inputMapping.end(), [parentKeyOrConstant, childKey](Mapping& entry) {
        return entry.parentKeyOrConstant.index() == 0 && parentKeyOrConstant.index() == 0 &&
               std::get<0>(entry.parentKeyOrConstant) == std::get<0>(parentKeyOrConstant) && entry.childKey == childKey;
    }) == 0);
    _inputMapping.push_back({parentKeyOrConstant, childKey});
}

void KeyMapping::addOutputMapping(const std::string& parentKey, const std::string& childKey)
{
    // output mapping does not support constant values, so we pass an empty string as value
    std::variant<std::string, std::string> parentKeyVar;
    parentKeyVar.emplace<0>(parentKey);

    assert(std::count_if(_outputMapping.begin(), _outputMapping.end(), [parentKeyVar, childKey](Mapping& entry) {
        return entry.parentKeyOrConstant.index() == 0 && std::get<0>(entry.parentKeyOrConstant) == std::get<0>(parentKeyVar) && entry.childKey == childKey;
    }) == 0);

    _outputMapping.push_back({parentKeyVar, childKey});
}

} /* namespace alica */
