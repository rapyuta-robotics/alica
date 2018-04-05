#pragma once
#include "AlicaElement.h"

#include "supplementary/AgentID.h"
#include "Plan.h"
#include "EntryPoint.h"
#include "State.h"

#include <string>
#include <vector>

namespace alica {
class State;
class EntryPoint;
class Plan;
class Variable;
class RunningPlan;
class AlicaEngine;
class SolverTerm;
class ModelFactory;
/**
 * A quantifier encapsulates a set of Variables, belonging to a domain artifact, scoped under a AlicaElement
 */
class Quantifier : public AlicaElement {
public:
    Quantifier(int64_t id = 0);
    virtual ~Quantifier();
    const std::vector<std::string>& getDomainIdentifiers() const {return _domainIdentifiers;}
    bool isScopeIsEntryPoint() const {return _scopeType == entryPointScope;}
    bool isScopeIsPlan() const {return _scopeType == planScope;}
    bool isScopeIsState() const {return _scopeType == stateScope;}
    const State* getScopedState()  const {return _scopeType == stateScope ? static_cast<const State*>(_scope) : nullptr;}
    const EntryPoint* getScopedEntryPoint() const {return _scopeType == entryPointScope ? static_cast<const EntryPoint*>(_scope) : nullptr;}
    const Plan* getScopedPlan() const {return _scopeType == planScope ? static_cast<const Plan*>(_scope) : nullptr;}
    const AlicaElement* getScope() const {return _scope;}
    
    /**
     * Access the std::list of sorted Variables under the scope of this quantifier given a runningplan.
     * @param p A RunningPlan
     * @param agentsInScope A std::shared_ptr<std::vector<int> >
     * @return A std::shared_ptr<std::list<std::vector<Variable* > > >
     */
    virtual std::shared_ptr<std::list<std::vector<Variable*>>> getDomainVariables(std::shared_ptr<RunningPlan>& p,
            std::shared_ptr<std::vector<const supplementary::AgentID*>>& agentsInScope) = 0;

private:
    enum Scope {
        planScope, entryPointScope, stateScope
    };
    friend ModelFactory;
    void setScope(const AlicaElement* ae);
    void setScopeType(Scope scope);
    void setDomainIdentifiers(const std::vector<std::string>& domainIdentifiers);

    std::vector<std::string> _domainIdentifiers;

    const AlicaElement* _scope; 
    Scope _scopeType;
};

}  // namespace alica
