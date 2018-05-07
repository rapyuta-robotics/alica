#pragma once

#include <list>
#include <memory>
#include <vector>
#include <engine/Types.h>

namespace alica {
class Condition;
class RunningPlan;
class Variable;

class ProblemPart {
public:
    ProblemPart(const Condition* con, std::shared_ptr<RunningPlan> rp);

    bool hasVariable(const Variable* v) const;

    const Condition* getCondition() const;
    std::shared_ptr<std::vector<std::list<VariableGrp>>> getDomainVariables() const;
    std::shared_ptr<RunningPlan> getRunningPlan() const;
    std::shared_ptr<std::vector<std::shared_ptr<AgentGrp>>> getAgentsInScope() const;

private:
    const Condition* condition;
    /**
     *  Hierarchie: 1.vector< 2.list< 3.vector< 4.Variable* > > >
     * 1. Vector of Quantors, e.g., For all agents in state S variables X,Y exist.
     * 2. List of Robots, e.g., An agent has variables X,Y.
     * 3. Vector of Variables, e.g., variables X,Y.
     * 4. Variable, e.g., variable X.
     */
    std::shared_ptr<std::vector<std::list<VariableGrp>>> domainVariables;
    std::shared_ptr<RunningPlan> runningplan;
    std::shared_ptr<std::vector<std::shared_ptr<AgentGrp>>> agentsInScope;
};

} /* namespace alica */
