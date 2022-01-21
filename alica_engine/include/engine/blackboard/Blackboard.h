#pragma once

#include <any>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <type_traits>
#include <unordered_map>

namespace alica
{
class BlackboardImpl
{
public:
    // Not for API use, but public to allow modifying without mutex when we know the behavior/plan is not running
    template <class... Args>
    void registerValue(const std::string& key, Args&&... args)
    {
        vals.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(std::forward<decltype(args)>(args)...));
    }

    template <typename T>
    const T& get(const std::string& key) const
    {
        return std::any_cast<const T&>(vals.at(key));
    }
    template <typename T>
    T& get(const std::string& key)
    {
        return std::any_cast<T&>(vals.at(key));
    }
    std::any& get(const std::string& key) { return vals.at(key); }
    const std::any& get(const std::string& key) const { return vals.at(key); }

    void set(const std::string& key, const std::any& value) { vals.at(key) = value; }

    bool hasValue(const std::string& key) const { return vals.count(key); }
    void removeValue(const std::string& key) { vals.erase(key); }

    void clear() { vals.clear(); }
    bool empty() const { return vals.empty(); }
    size_t size() const { return vals.size(); }
    std::unordered_map<std::string, std::any> vals;
};

class Blackboard
{
public:
    Blackboard() = default;
    Blackboard(Blackboard&&) = delete;
    Blackboard& operator&=(const Blackboard&) = delete;
    Blackboard& operator&=(Blackboard&&) = delete;

    std::shared_lock<std::shared_mutex> lockRO() const { return std::shared_lock(_mtx); }
    std::unique_lock<std::shared_mutex> lockRW() { return std::unique_lock(_mtx); }

    // Not thread safe.  Avoid for public use
    BlackboardImpl& impl() { return _impl; }
    const BlackboardImpl& impl() const { return _impl; }

private:
    BlackboardImpl _impl;
    mutable std::shared_mutex _mtx;
};

class LockedBlackboardRO
{
public:
    LockedBlackboardRO(const Blackboard& bb)
            : _lk(bb.lockRO())
            , _impl(&bb.impl())
    {
    }
    LockedBlackboardRO& operator&=(const LockedBlackboardRO&) = delete;
    LockedBlackboardRO& operator&=(LockedBlackboardRO&) = delete;
    LockedBlackboardRO(LockedBlackboardRO&) = delete;

    bool empty() const { return _impl->empty(); }
    size_t size() const { return _impl->size(); }
    template <typename T>
    const T& get(const std::string& key) const
    {
        return _impl->get<T>(key);
    }
    const std::any& get(const std::string& key) const { return _impl->get(key); }
    bool hasValue(const std::string& key) const { return _impl->hasValue(key); }

private:
    std::shared_lock<std::shared_mutex> _lk;
    const BlackboardImpl* _impl;
};

class LockedBlackboardRW
{
public:
    LockedBlackboardRW(Blackboard& bb)
            : _lk(bb.lockRW())
            , _impl(&bb.impl())
    {
    }
    LockedBlackboardRW& operator&=(const LockedBlackboardRW&) = delete;
    LockedBlackboardRW& operator&=(LockedBlackboardRW&) = delete;
    LockedBlackboardRW(LockedBlackboardRW&) = delete;

    void clear() { _impl->clear(); }

    void removeValue(const std::string& key) { _impl->removeValue(key); }

    template <class... Args>
    void registerValue(const std::string& key, Args&&... args)
    {
        _impl->registerValue(key, std::forward<Args>(args)...);
    }

    template <typename T>
    T& get(const std::string& key)
    {
        return _impl->get<T>(key);
    }
    std::any& get(const std::string& key) { return _impl->get(key); }

    void set(const std::string& key, const std::any& value) { _impl->set(key, value); }

    bool empty() const { return _impl->empty(); }
    size_t size() const { return _impl->size(); }
    bool hasValue(const std::string& key) const { return _impl->hasValue(key); }

private:
    std::unique_lock<std::shared_mutex> _lk;
    BlackboardImpl* _impl;
};

} // namespace alica
