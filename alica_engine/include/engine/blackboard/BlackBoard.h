#pragma once

#include <any>
#include <unordered_map>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <type_traits>

namespace alica
{
class BlackBoardImpl
{
public:
    // Not for API use, but public to allow modifying without mutex when we know the behavior/plan is not running
    template <class... Args> void registerValue(const std::string& key, Args&&... args) { 
        vals.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(std::forward<decltype(args)>(args)...)); 
    }
    
    template <typename T> const T& get(const std::string& key) const {
        return std::any_cast<const T&>(vals.at(key));
    }
    template <typename T> T& get(const std::string& key) {
        return std::any_cast<T&>(vals.at(key));
    }

    bool hasValue(const std::string& key) const {
        return vals.count(key);
    }
    void removeValue(const std::string& key) {
        vals.erase(key);
    }

    void clear() {
        vals.clear();
    }
    bool empty() const {
        return vals.empty();
    }
    size_t size() const {
        return vals.size();
    }
    std::unordered_map<std::string, std::any> vals;
};

class BlackBoard
{
public:
    BlackBoard() = default;
    BlackBoard(BlackBoard&&) = delete;
    BlackBoard& operator&=(const BlackBoard&) = delete;
    BlackBoard& operator&=(BlackBoard&&) = delete;

    std::shared_lock<std::shared_mutex> lockRO() const {return std::shared_lock(_mtx);}
    std::unique_lock<std::shared_mutex> lockRW() {return std::unique_lock(_mtx);}

    // Not thread safe.  Avoid for public use
    BlackBoardImpl& impl() {return _impl;}
    const BlackBoardImpl& impl() const {return _impl;}
private:
    BlackBoardImpl _impl;
    mutable std::shared_mutex _mtx;
};

class LockedBlackBoardRO
{
public:
    LockedBlackBoardRO(const BlackBoard& bb) : 
         _lk(bb.lockRO())
      ,  _impl(&bb.impl())
    {}

    bool empty() const {
        return _impl->empty();
    }
    size_t size() const {
        return _impl->size();
    }
    template <typename T> const T& get(const std::string& key) {
        return _impl->get<T>(key);
    }
    bool hasValue(const std::string& key) const {
        return _impl->hasValue(key);
    }
private:
    std::shared_lock<std::shared_mutex> _lk;
    const BlackBoardImpl* _impl;
};

class LockedBlackBoardRW
{
public:
    LockedBlackBoardRW(BlackBoard& bb) : 
         _lk(bb.lockRW())
      ,  _impl(&bb.impl())
    {}

    void clear() {
        _impl->clear();
    }

    void removeValue(const std::string& key) {
        _impl->removeValue(key);
    }

    template <class... Args> void registerValue(const std::string& key, Args&&... args) { 
        _impl->registerValue(key, std::forward<Args>(args)...);
    }

    template <typename T> T& get(const std::string& key) {
        return _impl->get<T>(key);
    }
    bool empty() const {
        return _impl->empty();
    }
    size_t size() const {
        return _impl->size();
    }
    bool hasValue(const std::string& key) const {
        return _impl->hasValue(key);
    }
private:
    std::unique_lock<std::shared_mutex> _lk;
    BlackBoardImpl* _impl;
};

} // namespace alica