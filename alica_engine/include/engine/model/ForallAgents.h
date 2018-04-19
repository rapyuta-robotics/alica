#pragma once

#include "Quantifier.h"
#include "supplementary/AgentID.h"
#include <engine/Types.h>

#include <list>
#include <memory>
#include <vector>

namespace alica {

class RunningPlan;
class Variable;
class SolverTerm;

/**
 * A quantifier associated with agents, i.e., the domain identifiers of this quantifier refer to properties of an agent
 */
class ForallAgents : public Quantifier {
public:
    ForallAgents(int64_t id = 0);
    virtual ~ForallAgents();
    virtual bool isAgentUnderScope(AgentIdPtr id, std::shared_ptr<RunningPlan>& rp) const override;
    virtual bool addDomainVariables(std::shared_ptr<RunningPlan>& p, std::vector<AgentVariables>& io_agentVarsInScope) const override;
};

}  // namespace alica
