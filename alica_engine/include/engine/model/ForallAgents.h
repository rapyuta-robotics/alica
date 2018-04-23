#pragma once

#include "Quantifier.h"
#include "supplementary/AgentID.h"
#include <engine/Types.h>
#include <engine/collections/AgentVariables.h>

#include <list>
#include <memory>
#include <vector>

namespace alica
{

class RunningPlan;
class Variable;
class SolverTerm;
class TeamManager;

/**
 * A quantifier associated with agents, i.e., the domain identifiers of this quantifier refer to properties of an agent
 */
class ForallAgents : public Quantifier
{
  public:
    ForallAgents(int64_t id = 0);
    virtual ~ForallAgents();
    virtual bool isAgentInScope(AgentIDPtr id, const std::shared_ptr<const RunningPlan>& rp) const override;
    virtual bool addDomainVariables(const std::shared_ptr<const RunningPlan>& p, std::vector<AgentVariables>& io_agentVarsInScope) const override;

  private:
    enum Result
    {
        ADDED,
        MODIFIED,
        NONE
    };
    Result TryAddId(AgentIDPtr id, std::vector<AgentVariables>& io_agentVarsInScope, const TeamManager* tm) const;
};

} // namespace alica
