#pragma once

#include <string>
#include <vector>

namespace alica
{

class Blackboard;

class KeyMapping
{
    struct Mapping
    {
        Mapping(const std::string& src, const std::string& target);
        std::string src, target;
    };

public:
    using KeyMappingList = std::vector<Mapping>;
    const KeyMappingList& getInputMapping() const;
    const KeyMappingList& getInputValueMapping() const;
    const KeyMappingList& getOutputMapping() const;
    void addInputMapping(const std::string& parentKey, const std::string& childKey);
    void addInputValueMapping(const std::string& value, const std::string& childKey);
    void addOutputMapping(const std::string& parentKey, const std::string& childKey);

private:
    KeyMappingList _inputMapping;
    KeyMappingList _inputValueMapping;
    KeyMappingList _outputMapping;
};

} /* namespace alica */
