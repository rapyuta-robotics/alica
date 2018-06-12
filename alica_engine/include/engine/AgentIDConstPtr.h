#pragma once

#include <supplementary/AgentID.h>
namespace alica
{

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

    bool operator==(const AgentIDConstPtr o) const { return _ptr == o._ptr || (_ptr != nullptr && o._ptr != nullptr && *_ptr == *o._ptr); }
    bool operator!=(const AgentIDConstPtr id) const { return !AgentIDConstPtr::operator==(id); }
    bool operator<(const AgentIDConstPtr id) const { return *_ptr < *id._ptr; }
    const supplementary::AgentID* get() const { return _ptr; }

private:
    friend std::ostream& operator<<(std::ostream& out, const AgentIDConstPtr a);
    const supplementary::AgentID* _ptr;
};

std::ostream& operator<<(std::ostream& out, const AgentIDConstPtr a)
{
    if (a) {
        out << *a;
    } else {
        out << "NULL";
    }
    return out;
}
}