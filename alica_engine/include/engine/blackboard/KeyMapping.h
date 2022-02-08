#pragma once

#include <string>
#include <vector>

namespace alica
{

class Blackboard;

class KeyMapping
{
public:
    using KeyMappingList = std::vector<std::pair<std::string, std::string>>;
    KeyMapping() = default;
    virtual ~KeyMapping() = default;
    const KeyMappingList& getInputMapping() const;
    const KeyMappingList& getOutputMapping() const;
    void addInputMapping(const std::string& parentKey, const std::string& childKey);
    void addOutputMapping(const std::string& parentKey, const std::string& childKey);

private:
    KeyMappingList _inputMapping;
    KeyMappingList _outputMapping;
};

} /* namespace alica */
