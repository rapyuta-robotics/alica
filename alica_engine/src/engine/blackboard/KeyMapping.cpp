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

KeyMapping::KeyMappingList KeyMapping::getInputMapping() const
{
    return _inputMapping;
}

KeyMapping::KeyMappingList KeyMapping::getOutputMapping() const
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

} /* namespace alica */
