#include "engine/constraintmodul/ProblemPart.h"

#include "engine/RunningPlan.h"
#include "engine/model/Condition.h"
#include "engine/model/Quantifier.h"

namespace alica {

ProblemPart::ProblemPart(const Condition* con, shared_ptr<RunningPlan> rp) {
    condition = con;
    domainVariables = std::make_shared<std::vector<std::list<VariableGrp>>>();
    agentsInScope = std::make_shared<std::vector<std::shared_ptr<AgentGrp>>>();
    for (const Quantifier* quantifier : condition->getQuantifiers()) {
        std::shared_ptr<AgentGrp> robots = std::make_shared<AgentGrp>();
        domainVariables->push_back(*quantifier->getDomainVariables(rp, *robots));
    }
    runningplan = rp;
}

/**
 * Checks whether the given variable is one of the domain variables.
 */
bool ProblemPart::hasVariable(const Variable* v) const {
    for (auto& listOfRobots : (*domainVariables)) {
        for (const VariableGrp& variables : listOfRobots) {
            for (const Variable* variable : variables) {
                if (variable == v) {
                    return true;
                }
            }
        }
    }
    return false;
}

const Condition* ProblemPart::getCondition() const {
    return condition;
}

/**
 *  Hierarchie: 1.vector< 2.list< 3.vector< 4.Variable* > > >
 * 1. Vector of Quantors, e.g., For all agents in state S variables X,Y exist.
 * 2. List of Robots, e.g., An agent has variables X,Y.
 * 3. Vector of Variables, e.g., variables X,Y.
 * 4. Variable, e.g., variable X.
 */
std::shared_ptr<std::vector<std::list<VariableGrp>>> ProblemPart::getDomainVariables() const {
    return domainVariables;
}

std::shared_ptr<RunningPlan> ProblemPart::getRunningPlan() const {
    return runningplan;
}

std::shared_ptr<std::vector<std::shared_ptr<AgentGrp>>> ProblemPart::getAgentsInScope() const {
    return agentsInScope;
}

} /* namespace alica */
