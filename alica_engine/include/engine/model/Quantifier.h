#pragma once
#include "AlicaElement.h"

#include "supplementary/AgentID.h"

#include <list>
#include <memory>
#include <string>
#include <typeinfo>
#include <vector>

namespace alica {
class State;
class EntryPoint;
class Plan;
class Variable;
class RunningPlan;
class AlicaEngine;
class SolverTerm;

/**
 * A quantifier encapsulates a set of Variables, belonging to a domain artifact, scoped under a AlicaElement
 */
class Quantifier : public AlicaElement {
public:
    Quantifier(long id = 0);
    virtual ~Quantifier();
    std::list<std::string>& getDomainIdentifiers();
    void setDomainIdentifiers(const std::list<std::string>& domainIdentifiers);
    bool isScopeIsEntryPoint() const;
    bool isScopeIsPlan() const;
    bool isScopeIsState() const;
    State* getScopedState();
    EntryPoint* getScopedEntryPoint();
    Plan* getScopedPlan();
    void setScope(AlicaEngine* a, AlicaElement* ae);
    AlicaElement* getScope();
    /**
     * Access the std::list of sorted Variables under the scope of this quantifier given a runningplan.
     * @param p A RunningPlan
     * @param agentsInScope A std::shared_ptr<std::vector<int> >
     * @return A std::shared_ptr<std::list<std::vector<Variable* > > >
     */
    virtual std::shared_ptr<std::list<std::vector<Variable*>>> getDomainVariables(std::shared_ptr<RunningPlan>& p,
            std::shared_ptr<std::vector<const supplementary::AgentID*>>& agentsInScope) = 0;

private:
    std::list<std::string> domainIdentifiers;
    void setScopeIsEntryPoint(bool scopeIsEntryPoint);
    void setScopeIsPlan(bool scopeIsPlan);
    void setScopeIsState(bool scopeIsState);

protected:
    /**
     * Indicates that the scope of this quantifier is an EntryPoint
     */
    bool scopeIsEntryPoint;
    /**
     * Indicates that the scope of this quantifier is a Plan
     */
    bool scopeIsPlan;
    /**
     * Indicates that the scope of this quantifier is an State
     */
    bool scopeIsState;
    EntryPoint* entryPoint;
    State* state;
    Plan* plan;
};

}  // namespace alica
