#pragma once

#include <essentials/AgentID.h>
namespace alica
{

// The current AgentIDManager implementation allows for equality by pointer comparison
// but this only works as long as there is only one AgentIDManager.
// Note that the difference in performance is significant, because for the fast check, agent ids do not need to be in cache,
// whereas for the slow version, any find-in-datastructures operation will likely be stalled several times due to cache misses.
#define AGENT_ID_FAST_EQUALITY_CHECK

class AgentIDConstPtr
{
public:
    AgentIDConstPtr()
            : _ptr(nullptr)
    {
    }
    AgentIDConstPtr(const essentials::AgentID* id)
            : _ptr(id)
    {
    }
    const essentials::AgentID& operator*() const { return *_ptr; }
    const essentials::AgentID* operator->() const { return _ptr; }

    explicit operator bool() const { return _ptr != nullptr; }
#ifdef AGENT_ID_FAST_EQUALITY_CHECK
    bool operator==(const AgentIDConstPtr o) const { return _ptr == o._ptr; }
    std::size_t hash() const { return reinterpret_cast<size_t>(_ptr); }
#else
    bool operator==(const AgentIDConstPtr o) const { return _ptr == o._ptr || (_ptr != nullptr && o._ptr != nullptr && *_ptr == *o._ptr); }
    std::size_t hash() const { return _ptr->hash(); }
#endif
    bool operator!=(const AgentIDConstPtr id) const { return !AgentIDConstPtr::operator==(id); }
    bool operator<(const AgentIDConstPtr id) const { return *_ptr < *id._ptr; }
    const essentials::AgentID* get() const { return _ptr; }

private:
    friend std::ostream& operator<<(std::ostream& out, const AgentIDConstPtr a);
    const essentials::AgentID* _ptr;
};

inline std::ostream& operator<<(std::ostream& out, const AgentIDConstPtr a)
{
    if (a) {
        out << *a;
    } else {
        out << "NULL";
    }
    return out;
}
}

namespace std
{
template <>
struct hash<alica::AgentIDConstPtr>
{
    std::size_t operator()(const alica::AgentIDConstPtr id) const noexcept { return id.hash(); }
};
}

// backwards compatibility:
namespace alica
{

using AgentIDHash = std::hash<AgentIDConstPtr>;
}
