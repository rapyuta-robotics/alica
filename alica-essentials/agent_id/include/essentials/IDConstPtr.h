#pragma once

#include "essentials/ID.h"

namespace essentials
{

// The current AgentIDManager implementation allows for equality by pointer comparison
// but this only works as long as there is only one AgentIDManager.
// Note that the difference in performance is significant, because for the fast check, agent ids do not need to be in cache,
// whereas for the slow version, any find-in-datastructures operation will likely be stalled several times due to cache misses.
#define ID_FAST_EQUALITY_CHECK

class IDConstPtr
{
public:
    IDConstPtr()
            : _ptr(nullptr)
    {
    }
    IDConstPtr(const ID* id)
            : _ptr(id)
    {
    }
    const ID& operator*() const { return *_ptr; }
    const ID* operator->() const { return _ptr; }

    explicit operator bool() const { return _ptr != nullptr; }
#ifdef ID_FAST_EQUALITY_CHECK
    bool operator==(const IDConstPtr o) const { return _ptr == o._ptr; }
    std::size_t hash() const { return reinterpret_cast<size_t>(_ptr); }
#else
    bool operator==(const IDConstPtr o) const { return _ptr == o._ptr || (_ptr != nullptr && o._ptr != nullptr && *_ptr == *o._ptr); }
    std::size_t hash() const { return _ptr->hash(); }
#endif
    bool operator!=(const IDConstPtr id) const { return !IDConstPtr::operator==(id); }
    bool operator<(const IDConstPtr id) const { return *_ptr < *id._ptr; }
    const ID* get() const { return _ptr; }

private:
    friend std::ostream& operator<<(std::ostream& out, const IDConstPtr a);
    const ID* _ptr;
};

inline std::ostream& operator<<(std::ostream& out, const IDConstPtr a)
{
    if (a) {
        out << *a;
    } else {
        out << "NULL";
    }
    return out;
}

struct IDConstPtrHash
{
    std::size_t operator()(const IDConstPtr obj) const { return obj.hash(); }
};
}

namespace std
{
template <>
struct hash<essentials::IDConstPtr>
{
    std::size_t operator()(const essentials::IDConstPtr id) const noexcept { return id.hash(); }
};
}
