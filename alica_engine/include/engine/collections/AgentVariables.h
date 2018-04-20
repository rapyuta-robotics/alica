#pragma once

#include "engine/Types.h"
#include "engine/constraintmodul/SolverVariable.h"

#include <memory>

namespace alica {

template <class T>
class AgentElements {
public:
    AgentElements(AgentIDPtr id)
            : _id(id) {}

    AgentElements(const AgentElements&) = delete;
    AgentElements& operator=(const AgentElements&) = delete;

    AgentElements(const AgentElements&& o)
            : _id(o._id)
            , _vars(std::move(o._vars)) {}

    AgentElements& operator=(AgentElements&& o) {
        std::swap(_id, o._id);
        _vars = std::move(o._vars);
        return *this;
    }
    AgentIDPtr getId() const { return _id; }

    const std::vector<T>& getVars() const { return _vars; }
    std::vector<T>& editVars() { return _vars; }

    bool operator<(const AgentElements& o) const { return *_id < o._id; }
    bool operator>(const AgentElements& o) const { return *_id > o._id; }

    bool operator<=(const AgentElements& o) const { return *_id <= o._id; }
    bool operator>=(const AgentElements& o) const { return *_id >= o._id; }

private:
    AgentIDPtr _id;
    std::vector<T> _vars;
};

using AgentVariables = AgentElements<const DomainVariable*>;

struct RangedVariable {
    RangedVariable(const std::shared_ptr<SolverVariable>& sv)
            : var(sv)
            , min(SolverVariable::minExpressibleValue)
            , max(SolverVariable::maxExpressibleValue) {}
    std::shared_ptr<SolverVariable> var;
    double min;
    double max;
};
using AgentSolverVariables = AgentElements<RangedVariable>;

}  // namespace alica