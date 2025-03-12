#pragma once

#include "engine/logging/Logging.h"
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

    bool operator==(const BlackboardBlueprint& other) const { return compare(*this, other, false); }

    bool operator!=(const BlackboardBlueprint& other) const { return !(*this == other); }

    static bool compare(const BlackboardBlueprint& lhs, const BlackboardBlueprint& rhs, bool excludeProtectedKeys)
    {
        for (auto& [key, info] : lhs._keyInfo) {
            if (excludeProtectedKeys && info.access == "protected") {
                continue;
            }
            if (auto it = rhs._keyInfo.find(key); it == rhs._keyInfo.end() || it->second.type != info.type || it->second.access != info.access) {
                Logging::logError("BlackboardBlueprint") << "Key: " << key << " , access: " << info.access << " , type: " << info.type << " mismatch";
                return false;
            }
        }
        for (auto& [key, info] : rhs._keyInfo) {
            if (excludeProtectedKeys && info.access == "protected") {
                continue;
            }
            if (auto it = lhs._keyInfo.find(key); it == lhs._keyInfo.end() || it->second.type != info.type || it->second.access != info.access) {
                Logging::logError("BlackboardBlueprint") << "Key: " << key << " , access: " << info.access << " , type: " << info.type << " mismatch";
                return false;
            }
        }
        return true;
    }

private:
    std::unordered_map<std::string, KeyInfo> _keyInfo;
};

} // namespace alica
