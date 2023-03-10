#include <algorithm>
#include <any>
#include <assert.h>
#include <engine/blackboard/Blackboard.h>
#include <engine/blackboard/KeyMapping.h>
#include <iostream>
#include <sstream>

namespace alica
{

KeyMapping::Mapping::Mapping(const std::string& src_, const std::string& target_)
        : src(src_)
        , target(target_)
{
}

const KeyMapping::KeyMappingList& KeyMapping::getInputMapping() const
{
    return _inputMapping;
}

const KeyMapping::KeyMappingList& KeyMapping::getInputValueMapping() const
{
    return _inputValueMapping;
}

const KeyMapping::KeyMappingList& KeyMapping::getOutputMapping() const
{
    return _outputMapping;
}

void KeyMapping::addInputMapping(const std::string& parentKey, const std::string& childKey)
{
    if (std::count_if(_inputMapping.begin(), _inputMapping.end(), [childKey](const auto& mapping) { return mapping.target == childKey; })) {
        throw std::logic_error("key: " + childKey + " already has an input mapping");
    }
    if (std::count_if(_inputValueMapping.begin(), _inputValueMapping.end(), [childKey](const auto& mapping) { return mapping.target == childKey; })) {
        throw std::logic_error("key: " + childKey + " already has an input value mapping");
    }
    _inputMapping.emplace_back(parentKey, childKey);
}

void KeyMapping::addInputValueMapping(const std::string& value, const std::string& childKey)
{
    if (std::count_if(_inputMapping.begin(), _inputMapping.end(), [childKey](const auto& mapping) { return mapping.target == childKey; })) {
        throw std::logic_error("key: " + childKey + " already has an input mapping");
    }
    if (std::count_if(_inputValueMapping.begin(), _inputValueMapping.end(), [childKey](const auto& mapping) { return mapping.target == childKey; })) {
        throw std::logic_error("key: " + childKey + " already has an input value mapping");
    }
    _inputValueMapping.emplace_back(value, childKey);
}

void KeyMapping::addOutputMapping(const std::string& parentKey, const std::string& childKey)
{
    if (std::count_if(_outputMapping.begin(), _outputMapping.end(), [parentKey](const auto& mapping) { return mapping.target == parentKey; })) {
        throw std::logic_error("key: " + parentKey + " already has an output mapping");
    }
    _outputMapping.emplace_back(childKey, parentKey);
}

} /* namespace alica */
