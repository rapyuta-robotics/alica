#include "engine/constraintmodul/ProblemPart.h"

#include "engine/RunningPlan.h"
#include "engine/model/Condition.h"
#include "engine/model/Quantifier.h"

namespace alica {

ProblemPart::ProblemPart(const Condition* con, shared_ptr<RunningPlan> rp)
    : _condition(con)
    , _runningPlan(rp)
 {
    for (const Quantifier* quantifier : con->getQuantifiers()) {
        quantifier->addDomainVariables(rp, _vars);
    }
}
ProblemPart::ProblemPart(ProblemPart&& o)
    : _vars(std::move(o._vars))
    , _condition(o._condition)
    , _runningPlan(std::move(o._runningPlan))
    {}

ProblemPart& ProblemPart::operator=(ProblemPart&& o) {
    _vars = std::move(o._vars);
    _condition = o._condition;
    _runningPlan = std::move(o._runningPlan);
    return *this;
}

/**
 * Checks whether the given variable is one of the domain variables.
 */
bool ProblemPart::hasVariable(const Variable* v) const {
    for (auto& listOfRobots : (*domainVariables)) {
        for (const VariableSet& variables : listOfRobots) {
            for (const Variable* variable : variables) {
                if (variable == v) {
                    return true;
                }
            }
        }
    }
    return false;
}
shared_ptr<ProblemDescriptor> ProblemPart::generateProblemDescriptor() {
 const VariableSet& staticCondVariables = probPart->getCondition()->getVariables();

        shared_ptr<ProblemDescriptor> pd = std::make_shared<ProblemDescriptor>();
        // create a vector of solver variables from the static condition variables
        auto staticSolverVars = std::make_shared<std::vector<std::shared_ptr<SolverVariable>>>();
        staticSolverVars->reserve(staticCondVariables.size());
        for (const Variable* variable : staticCondVariables) {
            auto representingVariable = _uniqueVarStore.getRep(variable);
            if (representingVariable->getSolverVar() == nullptr) {
                representingVariable->setSolverVar(solver->createVariable(representingVariable->getId()));
            }

            staticSolverVars->push_back(representingVariable->getSolverVar());
        }

        // create a vector of solver variables from the domain variables of the currently iterated problem part
        auto domainSolverVars =
                make_shared<vector<shared_ptr<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>>>>();
        auto agentsInScope = make_shared<vector<shared_ptr<AgentSet>>>();
        for (int j = 0; j < static_cast<int>(probPart->getDomainVariables()->size()); ++j) {
            auto ll = make_shared<vector<shared_ptr<vector<shared_ptr<SolverVariable>>>>>();
            agentsInScope->push_back(probPart->getAgentsInScope()->at(j));
            domainSolverVars->push_back(ll);
            for (auto domainVars : probPart->getDomainVariables()->at(j)) {
                auto domainSolverVars = make_shared<vector<shared_ptr<SolverVariable>>>();
                domainSolverVars->reserve(domainVars.size());
                for (int i = 0; i < static_cast<int>(domainVars.size()); ++i) {
                    if (domainVars.at(i)->getSolverVar() == nullptr) {
                        domainVars.at(i)->setSolverVar(solver->createVariable(domainVars.at(i)->getId()));
                    }
                    domainSolverVars->push_back(domainVars.at(i)->getSolverVar());
                }
                ll->push_back(domainSolverVars);
            }
        }

        // fill a new problem descriptor and put it into the out-parameter "pds"
        auto pd = make_shared<ProblemDescriptor>(staticSolverVars, domainSolverVars);
        pd->setAgentsInScope(agentsInScope);
        probPart->getCondition()->getConstraint(
                pd, probPart->getRunningPlan());  // this insert the actual problem description
        return pd;
}
} /* namespace alica */
