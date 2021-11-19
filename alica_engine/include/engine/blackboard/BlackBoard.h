#pragma once

#include <any>
#include <map>
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

    template <typename T> const T& getValue(const std::string& key) {
        return std::any_cast<const T&>(vals.at(key));
    }
    bool hasValue(const std::string& key) {
        return vals.count(key);
    }
    void removeValue(const std::string& key) { vals.erase(key); }

    void clear() { vals.clear(); }
    bool empty() const { return vals.empty(); }
    size_t size() const { return vals.size(); }

    BlackBoard(const BlackBoard&) = delete;
    BlackBoard(BlackBoard&&) = delete;
    BlackBoard& operator&=(const BlackBoard&) = delete;
    BlackBoard& operator&=(BlackBoard&&) = delete;

private:
    std::map<std::string, std::any> vals;
};

} // namespace alica