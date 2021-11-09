#pragma once

#include "Quantifier.h"

#include <engine/Types.h>
#include <engine/collections/AgentVariables.h>

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
    ForallAgents();
    virtual ~ForallAgents();
    bool isAgentInScope(uint64_t id, const RunningPlan& rp) const override;
    bool addDomainVariables(const RunningPlan& p, std::vector<AgentVariables>& io_agentVarsInScope) const override;

private:
    enum Result
    {
        ADDED,
        MODIFIED,
        NONE
    };
    Result TryAddId(uint64_t id, std::vector<AgentVariables>& io_agentVarsInScope, const TeamManager& tm) const;
};

} // namespace alica
