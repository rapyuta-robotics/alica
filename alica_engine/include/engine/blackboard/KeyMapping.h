#pragma once

#include <string>
#include <vector>

namespace alica
{

class Blackboard;

class KeyMapping
{
public:
    KeyMapping() = default;
    virtual ~KeyMapping() = default;
    void setInput(const Blackboard* parent_bb, Blackboard* child_bb) const;
    void setOutput(Blackboard* parent_bb, const Blackboard* child_bb) const;
    void addInputMapping(const std::string& parentKey, const std::string& childKey);
    void addOutputMapping(const std::string& parentKey, const std::string& childKey);

private:
    std::vector<std::pair<std::string, std::string>> inputMapping;
    std::vector<std::pair<std::string, std::string>> outputMapping;
};

} /* namespace alica */
