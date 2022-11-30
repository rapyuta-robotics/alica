#pragma once

#include <optional>
#include <string>
#include <unordered_map>

namespace alica
{

class BlackboardBlueprint
{
    struct KeyInfo
    {
        std::string type;
        std::optional<std::string> defaultValue;
    };

public:
    using const_iterator = std::unordered_map<std::string, KeyInfo>::const_iterator;

    void addKey(const std::string& key, const std::string& type, std::optional<std::string> defaultValue)
    {
        _keyInfo.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(KeyInfo{.type = type, .defaultValue = defaultValue}));
    }
    const_iterator begin() const { return _keyInfo.begin(); }
    const_iterator end() const { return _keyInfo.end(); }

private:
    std::unordered_map<std::string, KeyInfo> _keyInfo;
};

} // namespace alica
