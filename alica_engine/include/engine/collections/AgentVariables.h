#pragma once

#include "engine/Types.h"
namespace alica {

template <class T>
class AgentElements {
    AgentElements(AgentIDPtr id)
        : _id(id)
        {}
 
    AgentElements(const AgentElements&) = delete;
    AgentElements& operator=(const AgentElements&) = delete;

    AgentElements(const AgentElements&& o)
        : _id(o._id)
        , _var(std::move(o._vars))
    {}

    AgentElements& operator=(AgentElements&& o) {
        std::swap(_id,o._id);
        _vars = std::move(o._vars);
        return *this;
    }
    AgentIdPtr getId() const {return _id;}
 
    const std::vector<T>& getVars() const {return _vars;}
    std::vector<T>& editVars() {return _vars;}
 
    bool operator<(const AgentVariables& o) const {return *_id < o._id;}
    bool operator>(const AgentVariables& o) const {return *_id > o._id;}

    bool operator<=(const AgentVariables& o) const {return *_id <= o._id;}
    bool operator>=(const AgentVariables& o) const {return *_id >= o._id;}

    private:
    AgentIDPtr _id;
    std::vector<T> _vars;
};
struct RangedVariable {
    std::shared_ptr<SolverVariable> var;
    double min;
    double max;
};
using AgentVariables = AgentElements<const DomainVariable*>;
using AgentSolverVariables = AgentElements<RangedVariable>;

}