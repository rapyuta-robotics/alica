#pragma once

#include <supplementary/AgentID.h>
namespace alica
{

// The current AgentIDManager implementation allows for equality by pointer comparison
// but this only works as long as there is only one AgentIDManager.
// Note that the difference in performance is significant, because for the fast check, agent ids do not need to be in cache,
// whereas for the slow version, any find-in-datastructues operation will likely be stalled several times due to cache misses.
#define FAST_EQUALITY_CHECK

class AgentIDConstPtr
{
public:
    AgentIDConstPtr()
            : _ptr(nullptr)
    {
    }
    AgentIDConstPtr(const supplementary::AgentID* id)
            : _ptr(id)
    {
    }
    const supplementary::AgentID& operator*() const { return *_ptr; }
    const supplementary::AgentID* operator->() const { return _ptr; }

    operator bool() const { return _ptr != nullptr; }
#ifdef FAST_EQUALITY_CHECK
    bool operator==(const AgentIDConstPtr o) const { return _ptr == o._ptr; }
#else
    bool operator==(const AgentIDConstPtr o) const { return _ptr == o._ptr || (_ptr != nullptr && o._ptr != nullptr && *_ptr == *o._ptr); }
#endif
    bool operator!=(const AgentIDConstPtr id) const { return !AgentIDConstPtr::operator==(id); }
    bool operator<(const AgentIDConstPtr id) const { return *_ptr < *id._ptr; }
    const supplementary::AgentID* get() const { return _ptr; }

private:
    friend std::ostream& operator<<(std::ostream& out, const AgentIDConstPtr a);
    const supplementary::AgentID* _ptr;
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