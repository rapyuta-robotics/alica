#pragma once

#include <any>
#include <map>
#include <mutex>
#include <string>
#include <type_traits>

namespace alica
{

class BlackBoard
{
public:
    BlackBoard() = default;

    void registerValue(const std::string& key, std::any any);
    void registerValue(const char* key, std::any any);

    template <typename T> const T getValue(const std::string& key) const {
        std::lock_guard<std::mutex> lk(_mtx);
        return std::any_cast<const T&>(vals.at(key));
    }
    bool hasValue(const std::string& key) const;
    void removeValue(const std::string& key);

    void clear();
    bool empty() const;
    size_t size() const;

private:
    std::unordered_map<std::string, std::any> vals;
    mutable std::mutex _mtx;
};

} // namespace alica