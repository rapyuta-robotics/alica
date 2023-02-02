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
               std::get<std::string>(entry.parentKeyOrConstant) == std::get<std::string>(parentKeyOrConstant) && entry.childKey == childKey;
    }) == 0);
    _inputMapping.push_back({parentKeyOrConstant, childKey});
}

void KeyMapping::addOutputMapping(const std::string& parentKey, const std::string& childKey)
{
    // output mapping does not support constant values, so we pass an empty string as value
    assert(std::count(_outputMapping.begin(), _outputMapping.end(), {{parentKey, ""}, childKey}) == 0);
    std::variant<std::string, std::string> var;
    var.emplace<0>(parentKey);
    _outputMapping.push_back({var, childKey});
}

} /* namespace alica */
