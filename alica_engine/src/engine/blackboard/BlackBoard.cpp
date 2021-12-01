

#include "engine/blackboard/BlackBoard.h"

namespace alica
{
void BlackBoard::registerValue(const std::string& key, std::any val)
{
    std::lock_guard<std::mutex> lk(_mtx);
    vals.emplace(key, val);
}

void BlackBoard::registerValue(const char* key, std::any val)
{
    std::lock_guard<std::mutex> lk(_mtx);
    vals.emplace(key, val);
}

bool BlackBoard::hasValue(const std::string& key) const{
    std::lock_guard<std::mutex> lk(_mtx);
    return vals.count(key);
}
void BlackBoard::removeValue(const std::string& key) { 
    std::lock_guard<std::mutex> lk(_mtx);
    vals.erase(key); 
}

void BlackBoard::clear() { 
    std::lock_guard<std::mutex> lk(_mtx);
    vals.clear(); 
}
bool BlackBoard::empty() const { 
    std::lock_guard<std::mutex> lk(_mtx);
    return vals.empty(); 
}
size_t BlackBoard::size() const { 
    std::lock_guard<std::mutex> lk(_mtx);
    return vals.size(); 
}
} // namespace alica