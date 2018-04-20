#pragma once
#include "AlicaElement.h"

#include "engine/Types.h"

#include "Plan.h"
#include "EntryPoint.h"
#include "State.h"
#include "engine/collections/AgentVariables.h"

#include <algorithm>
#include <string>

namespace alica {
class RunningPlan;
class SolverTerm;
class ModelFactory;

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
        return _scopeType == STATESCOPE ? static_cast<const State*>(_scope) : nullptr;
    }
    const EntryPoint* getScopedEntryPoint() const {
        return _scopeType == ENTRYPOINTSCOPE ? static_cast<const EntryPoint*>(_scope) : nullptr;
    }
    const Plan* getScopedPlan() const { return _scopeType == PLANSCOPE ? static_cast<const Plan*>(_scope) : nullptr; }
    const AlicaElement* getScope() const { return _scope; }
    const VariableSet& getTemplateVariables() const { return _templateVars; }
    bool hasTemplateVariable(const Variable* v) const {
        return std::find(_templateVars.begin(), _templateVars.end(), v) != _templateVars.end();
    }
    virtual bool isAgentInScope(AgentIDPtr id, std::shared_ptr<RunningPlan>& rp) const = 0;
    /**
     * Access the list of sorted Variables under the scope of this quantifier given a runningplan.
     * @param p A RunningPlan
     * @param io_agentsInScope the list of Agents with their variables that this quantifier will add to.
     * @return true if io_agentVarsInScop was modified, false otherwise
     */
    virtual bool addDomainVariables(
            std::shared_ptr<RunningPlan>& p, std::vector<AgentVariables>& io_agentVarsInScope) const = 0;

protected:
    enum Scope { PLANSCOPE, ENTRYPOINTSCOPE, STATESCOPE };
    Scope getScopeType() const { return _scopeType; }

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
