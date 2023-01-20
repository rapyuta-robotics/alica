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

void KeyMapping::addInputMapping(const std::string& parentKey, const std::string& childKey, const std::string& value)
{
    assert(std::count_if(_inputMapping.begin(), _inputMapping.end(), [parentKey, childKey](std::tuple<std::string, std::string, std::string>& entry) {
        return std::get<0>(entry) == parentKey && std::get<1>(entry) == childKey;
    }) == 0);
    _inputMapping.push_back(std::make_tuple(parentKey, childKey, value));
}

void KeyMapping::addOutputMapping(const std::string& parentKey, const std::string& childKey)
{
    // output mapping does not support constant values, so we pass an empty string as value
    assert(std::count(_outputMapping.begin(), _outputMapping.end(), std::make_tuple(parentKey, childKey, "")) == 0);
    _outputMapping.push_back(std::make_tuple(parentKey, childKey, ""));
}

} /* namespace alica */
