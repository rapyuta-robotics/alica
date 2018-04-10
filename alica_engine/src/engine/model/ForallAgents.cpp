#include "engine/model/ForallAgents.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/RunningPlan.h"
#include "engine/TeamObserver.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/StateCollection.h"
#include "engine/constraintmodul/SolverTerm.h"
#include "engine/constraintmodul/SolverVariable.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/model/Variable.h"
#include "engine/teammanager/Agent.h"
#include "engine/teammanager/TeamManager.h"

namespace alica {

ForallAgents::ForallAgents(int64_t id)
        : Quantifier(id) {}

ForallAgents::~ForallAgents() {}

/**
 * Returns the <see cref="Variable"/>s currently associated with the agents occupying the scope of this quantifier.
 * @param plan A RunningPlan
 * @param agentsInScope A shared_ptr<vector<int> >
 * @return shared_ptr<list<VariableSet> >
 */
shared_ptr<list<VariableSet>> ForallAgents::getDomainVariables(
        shared_ptr<RunningPlan>& p, AgentSet& o_agentsInScope) const {
    o_agentsInScope.clear();
    if (isScopePlan()) {
        if (p->getPlan() == getScopedPlan()) {
            p->getAssignment()->getAllRobotsSorted(o_agentsInScope);
        }
    } else if (isScopeEntryPoint()) {
        p->getAssignment()->getRobotsWorkingSorted(getScopedEntryPoint(), o_agentsInScope);
    } else if (isScopeState()) {
        p->getAssignment()->getRobotStateMapping()->getRobotsInStateSorted(getScopedState(), o_agentsInScope);
    }
    if (o_agentsInScope.empty()) {
        return nullptr;
    }
    auto ret = make_shared<list<VariableSet>>();
    auto tm = p->getAlicaEngine()->getTeamManager();
    for (auto& r : o_agentsInScope) {
        auto robotEngineData = tm->getAgentByID(r)->getEngineData();

        VariableSet terms;
        for (auto identifier : this->getDomainIdentifiers()) {
            terms.push_back(robotEngineData->getDomainVariable(identifier));
        }
        ret->push_back(terms);
    }
    return ret;
}
}  // namespace alica
