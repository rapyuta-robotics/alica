#pragma once

#include <any>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace alica
{
class BlackboardBlueprint
{
public:
    template <class... Args>
    void registerValue(const std::string& key, Args&&... args)
    {
        vals.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(std::forward<decltype(args)>(args)...));
    }
    friend class Blackboard;

private:
    std::unordered_map<std::string, std::any> vals;
};

} // namespace alica
