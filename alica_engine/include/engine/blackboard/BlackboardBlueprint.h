#pragma once

#include <iostream>
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
        std::string access;
    };

public:
    using const_iterator = std::unordered_map<std::string, KeyInfo>::const_iterator;

    void addKey(const std::string& key, const std::string& type, const std::string& access)
    {
        _keyInfo.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(KeyInfo{type, access}));
    }
    const_iterator begin() const { return _keyInfo.begin(); }
    const_iterator end() const { return _keyInfo.end(); }
    const_iterator find(const std::string& key) const { return _keyInfo.find(key); }
    friend std::ostream& operator<<(std::ostream& out, const BlackboardBlueprint& bpt)
    {
        if (bpt._keyInfo.size()) {
            out << "{\n";
            for (auto& [key, info] : bpt._keyInfo) {
                out << "\t" << key << " : " << info.type << ",\n";
            }
            out << "}";
        } else {
            out << "{}";
        }

        return out;
    }

private:
    std::unordered_map<std::string, KeyInfo> _keyInfo;
};

} // namespace alica
