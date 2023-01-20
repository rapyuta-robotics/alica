#pragma once

#include <string>
#include <tuple>
#include <vector>

namespace alica
{

class Blackboard;

class KeyMapping
{
public:
    // contains tuple of parentKey, childKey and value
    using KeyMappingList = std::vector<std::tuple<std::string, std::string, std::string>>;
    KeyMapping() = default;
    virtual ~KeyMapping() = default;
    const KeyMappingList& getInputMapping() const;
    const KeyMappingList& getOutputMapping() const;
    void addInputMapping(const std::string& parentKey, const std::string& childKey, const std::string& value = "");
    void addOutputMapping(const std::string& parentKey, const std::string& childKey);

private:
    KeyMappingList _inputMapping;
    KeyMappingList _outputMapping;
};

} /* namespace alica */
