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
        std::string defaultValue;
    };

public:
    using const_iterator = std::unordered_map<std::string, KeyInfo>::const_iterator;

    void addKey(const std::string& key, const std::string& type, const std::string& access, const std::string& defaultValue = std::string{})
    {
        _keyInfo.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(KeyInfo{type, access, defaultValue}));
    }
    const_iterator begin() const { return _keyInfo.begin(); }
    const_iterator end() const { return _keyInfo.end(); }
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

    bool operator==(const BlackboardBlueprint& other) const
    {
        for (auto& [key, info] : _keyInfo) {
            auto it = other._keyInfo.find(key);
            if (it == other._keyInfo.end() || it->second.type != info.type || it->second.access != info.access) {
                return false;
            }
        }
        for (auto& [key, info] : other._keyInfo) {
            if (auto it = _keyInfo.find(key); it == _keyInfo.end()) {
                return false;
            }
        }
        return true;
    }

    bool operator!=(const BlackboardBlueprint& other) const { return !(*this == other); }

private:
    std::unordered_map<std::string, KeyInfo> _keyInfo;
};

} // namespace alica
