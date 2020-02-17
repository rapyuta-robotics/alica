#pragma once

#include "essentials/Identifier.h"

#include <sstream>

namespace essentials
{

// The current IDManager implementation allows for equality by pointer comparison
// but this only works as long as there is only one IDManager.
// Note that the difference in performance is significant, because for the fast check, agent ids do not need to be in cache,
// whereas for the slow version, any find-in-datastructures operation will likely be stalled several times due to cache misses.
#define ID_FAST_EQUALITY_CHECK

class IdentifierConstPtr
{
public:
    IdentifierConstPtr()
            : _ptr(nullptr)
    {
    }
    IdentifierConstPtr(const Identifier* id)
            : _ptr(id)
    {
    }
    const Identifier& operator*() const { return *_ptr; }
    const Identifier* operator->() const { return _ptr; }

    explicit operator bool() const { return _ptr != nullptr; }
#ifdef ID_FAST_EQUALITY_CHECK
    bool operator==(const IdentifierConstPtr o) const { return _ptr == o._ptr; }
    std::size_t hash() const { return reinterpret_cast<size_t>(_ptr); }
#else
    bool operator==(const IdentifierConstPtr o) const { return _ptr == o._ptr || (_ptr != nullptr && o._ptr != nullptr && *_ptr == *o._ptr); }
    std::size_t hash() const { return _ptr->hash(); }
#endif
    bool operator!=(const IdentifierConstPtr id) const { return !IdentifierConstPtr::operator==(id); }
    bool operator<(const IdentifierConstPtr id) const { return *_ptr < *id._ptr; }
    std::string toString();
    const Identifier* get() const { return _ptr; }

private:
    friend std::ostream& operator<<(std::ostream& out, const IdentifierConstPtr a);
    const Identifier* _ptr;
};

inline std::string IdentifierConstPtr::toString()
{
    std::stringstream ss;
    ss << *this;
    return ss.str();
}

inline std::ostream& operator<<(std::ostream& out, const IdentifierConstPtr a)
{
    if (a) {
        out << *a;
    } else {
        out << "NULL";
    }
    return out;
}

struct IdentifierConstPtrHash
{
    std::size_t operator()(const IdentifierConstPtr obj) const { return obj.hash(); }
};
}

namespace std
{
template <>
struct hash<essentials::IdentifierConstPtr>
{
    std::size_t operator()(const essentials::IdentifierConstPtr id) const noexcept { return id.hash(); }
};
}
