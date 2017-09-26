#pragma once

#include "engine/IRobotID.h"
#include <list>
#include <memory>
#include <vector>

namespace alica
{
class Condition;
class RunningPlan;
class Variable;

class ProblemPart
{
  public:
    ProblemPart(Condition *con, std::shared_ptr<RunningPlan> rp);

    bool hasVariable(Variable *v);

    Condition *getCondition();
    std::shared_ptr<std::vector<std::list<std::vector<Variable *>>>> getDomainVariables();
    std::shared_ptr<RunningPlan> getRunningPlan();
    std::shared_ptr<std::vector<std::shared_ptr<std::vector<const alica::IRobotID *>>>> getAgentsInScope();

  private:
    Condition *condition;
    /**
     *  Hierarchie: 1.vector< 2.list< 3.vector< 4.Variable* > > >
     * 1. Vector of Quantors, e.g., For all agents in state S variables X,Y exist.
     * 2. List of Robots, e.g., An agent has variables X,Y.
     * 3. Vector of Variables, e.g., variables X,Y.
     * 4. Variable, e.g., variable X.
     */
    std::shared_ptr<std::vector<std::list<std::vector<Variable *>>>> domainVariables;
    std::shared_ptr<RunningPlan> runningplan;
    std::shared_ptr<std::vector<std::shared_ptr<std::vector<const alica::IRobotID *>>>> agentsInScope;
};

} /* namespace alica */
