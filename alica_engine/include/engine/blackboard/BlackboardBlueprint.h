#pragma once

#include <any>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <variant>
#include <yaml-cpp/yaml.h>

namespace alica
{

class BlackboardBlueprint
{
    using Types = std::variant<bool, int64_t, double, std::string, std::any>;

public:
    template <class... Args>
    void registerValue(const std::string& key, Args&&... args)
    {
        vals.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(std::forward<decltype(args)>(args)...));
    }
    void setBlackboardNode(const YAML::Node& node) { this->node = node; }
    friend class Blackboard;

private:
    std::unordered_map<std::string, Types> vals;
    YAML::Node node;
};

} // namespace alica
