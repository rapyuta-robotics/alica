#pragma once

#include <unordered_map>
#include <list>

namespace alica
{

template <class Key, class Value, class Hasher = std::hash<Value>>
class LRUCache
{
public:
    LRUCache(std::size_t lruSize) : _lruSize(lruSize) {}

    template <class... Args>
    Value& lookup(const Key& key, Args&&... args)
    {
        auto lruIt = _lookupTable.find(key);
        if (lruIt != _lookupTable.end()) {
            auto valueIt = lruIt->second;
            _lruStore.splice(_lruStore.begin(), _lruStore, valueIt);
            return *valueIt;
        }
        if (_lruSize > 0 && _lruStore.size() == _lruSize) {
            _lruStore.pop_back();
        }
        _lruStore.emplace_front(std::forward<Args>(args)...);
        auto valueIt = _lruStore.begin();
        _lookupTable.emplace(std::piecewise_construct, std::forward_as_tuple(key), std::forward_as_tuple(valueIt));
        return *valueIt;
    }

private:
    const std::size_t _lruSize;
    std::list<Value> _lruStore;
    std::unordered_map<Key, typename std::list<Value>::iterator, Hasher> _lookupTable;
};

}