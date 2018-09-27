#pragma once

#include "engine/Types.h"
#include <alica_solver_interface/Interval.h>
#include <alica_solver_interface/SolverVariable.h>
#include <supplementary/AgentID.h>

namespace alica
{

template <class T>
class AgentElements
{
  public:
    AgentElements(AgentIDConstPtr id)
        : _id(id)
    {
    }

    AgentElements(const AgentElements&) = delete;
    AgentElements& operator=(const AgentElements&) = delete;

    AgentElements(const AgentElements&& o)
        : _id(o._id)
        , _vars(std::move(o._vars))
    {
    }

    AgentElements& operator=(AgentElements&& o)
    {
        std::swap(_id, o._id);
        _vars = std::move(o._vars);
        return *this;
    }
    AgentIDConstPtr getId() const { return _id; }

    const std::vector<T>& getVars() const { return _vars; }
    std::vector<T>& editVars() { return _vars; }

    bool operator<(const AgentElements& o) const { return *_id < *o._id; }
    bool operator>(const AgentElements& o) const { return *_id > *o._id; }

  private:
    AgentIDConstPtr _id;
    std::vector<T> _vars;
};

using AgentVariables = AgentElements<const DomainVariable*>;
using AgentSolverVariables = AgentElements<SolverVariable*>;

} // namespace alica