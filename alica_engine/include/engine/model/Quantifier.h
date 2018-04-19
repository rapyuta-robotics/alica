#pragma once
#include "AlicaElement.h"

#include "supplementary/AgentID.h"
#include "engine/Types.h"

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
class SolverTerm;
class ModelFactory;
class AgentVariables;
/**
 * A quantifier encapsulates a set of Variables, belonging to a domain artifact, scoped under a AlicaElement
 */
class Quantifier : public AlicaElement {
public:
    Quantifier(int64_t id = 0);
    virtual ~Quantifier();
    const std::vector<std::string>& getDomainIdentifiers() const { return _domainIdentifiers; }
    bool isScopeEntryPoint() const { return _scopeType == ENTRYPOINTSCOPE; }
    bool isScopePlan() const { return _scopeType == PLANSCOPE; }
    bool isScopeState() const { return _scopeType == STATESCOPE; }
    const State* getScopedState() const {
        return _scopeType == stateScope ? static_cast<const State*>(_scope) : nullptr;
    }
    const EntryPoint* getScopedEntryPoint() const {
        return _scopeType == entryPointScope ? static_cast<const EntryPoint*>(_scope) : nullptr;
    }
    const Plan* getScopedPlan() const { return _scopeType == planScope ? static_cast<const Plan*>(_scope) : nullptr; }
    const AlicaElement* getScope() const { return _scope; }
    const VariableSet& getTemplateVariables() const {return _templateVars;}
    bool hasTemplateVariable(const Variable* v) const {return std::find(_templateVars.begin(), _templateVars.end(), v) != _templateVars.end();}
    virtual bool isAgentUnderScope(AgentIdPtr id, std::shared_ptr<RunningPlan>& rp) const = 0;
    /**
     * Access the list of sorted Variables under the scope of this quantifier given a runningplan.
     * @param p A RunningPlan
     * @param io_agentsInScope the list of Agents with their variables that this quantifier will add to.
     * @return true if io_agentVarsInScop was modified, false otherwise
     */  
    virtual bool addDomainVariables(std::shared_ptr<RunningPlan>& p, AgentVariables& io_agentVarsInScope) const = 0;

protected:
enum Scope { PLANSCOPE, ENTRYPOINTSCOPE, STATESCOPE };
    Scope getScopeType() const {return _scopeType;}

private:
    
    friend ModelFactory;
    void setScope(const AlicaElement* ae);
    void setDomainIdentifiers(const std::vector<std::string>& domainIdentifiers);

    VariableSet _templateVars;
    std::vector<std::string> _domainIdentifiers;
    
    const AlicaElement* _scope;
    Scope _scopeType;
};

}  // namespace alica
