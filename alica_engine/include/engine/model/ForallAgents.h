#pragma once

#include "Quantifier.h"
#include "supplementary/AgentID.h"

#include <list>
#include <memory>
#include <vector>

namespace alica
{

class RunningPlan;
class Variable;
class AlicaEngine;
class SolverTerm;

/**
 * A quantifier associated with agents, i.e., the domain identifiers of this quantifier refer to properties of an agent
 */
class ForallAgents : public Quantifier
{
  public:
    ForallAgents(AlicaEngine *ae, long id = 0);
    virtual ~ForallAgents();
    std::shared_ptr<std::list<std::vector<Variable *>>>
    getDomainVariables(std::shared_ptr<RunningPlan> &p,
                       std::shared_ptr<std::vector<const supplementary::AgentID *>> &agentsInScope);

  protected:
    AlicaEngine *ae;
};

} /* namespace Alica */
