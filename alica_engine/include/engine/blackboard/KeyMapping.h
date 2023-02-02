#pragma once

#include <string>
#include <tuple>
#include <variant>
#include <vector>

namespace alica
{

class Blackboard;

struct Mapping
{
    std::variant<std::string, std::string> parentKeyOrConstant;
    std::string childKey;
};

class KeyMapping
{
public:
    using KeyMappingList = std::vector<Mapping>;
    KeyMapping() = default;
    virtual ~KeyMapping() = default;
    const KeyMappingList& getInputMapping() const;
    const KeyMappingList& getOutputMapping() const;
    void addInputMapping(const std::variant<std::string, std::string>& parentKeyOrConstant, const std::string& childKey);
    void addOutputMapping(const std::string& parentKey, const std::string& childKey);

private:
    KeyMappingList _inputMapping;
    KeyMappingList _outputMapping;
};

} /* namespace alica */
