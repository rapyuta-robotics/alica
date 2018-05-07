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
    virtual std::shared_ptr<std::list<VariableGrp>> getDomainVariables(
            std::shared_ptr<RunningPlan>& p, AgentGrp& o_agentsInScope) const override;
};

}  // namespace alica
