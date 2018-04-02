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

ForallAgents::ForallAgents(AlicaEngine* ae, long id) : Quantifier(id) {
    this->ae = ae;
}

ForallAgents::~ForallAgents() {}

/**
 * Returns the <see cref="Variable"/>s currently associated with the agents occupying the scope of this quantifier.
 * @param plan A RunningPlan
 * @param agentsInScope A shared_ptr<vector<int> >
 * @return shared_ptr<list<vector<Variable*> > >
 */
shared_ptr<list<vector<Variable*>>> ForallAgents::getDomainVariables(
        shared_ptr<RunningPlan>& p, shared_ptr<vector<const supplementary::AgentID*>>& agentsInScope) {
    if (this->isScopeIsPlan()) {
        if (p->getPlan() == this->getScopedPlan()) {
            agentsInScope = p->getAssignment()->getAllRobotsSorted();
        }
    } else if (this->isScopeIsEntryPoint()) {
        agentsInScope = p->getAssignment()->getRobotsWorkingSorted(this->getScopedEntryPoint());
    } else if (this->isScopeIsState()) {
        agentsInScope = p->getAssignment()->getRobotStateMapping()->getRobotsInStateSorted(this->getScopedState());
    }
    if (agentsInScope == nullptr) {
        return nullptr;
    }
    auto ret = make_shared<list<vector<Variable*>>>();
    auto tm = ae->getTeamManager();
    for (auto& r : *(agentsInScope)) {
        auto robotEngineData = tm->getAgentByID(r)->getEngineData();

        vector<Variable*> terms;
        for (auto identifier : this->getDomainIdentifiers()) {
            terms.push_back(robotEngineData->getDomainVariable(identifier));
        }
        ret->push_back(terms);

        //        vector<Variable *> terms = vector<Variable *>(this->getDomainIdentifiers().size());
        //        auto robotEngineData = tm->getAgentByID(r)->getEngineData();
        //        for (int i = 0; i < terms.size(); i++)
        //        {
        //            auto iter = this->getDomainIdentifiers().begin();
        //            advance(iter, i);
        //            terms[i] = robotEngineData->getDomainVariable(*iter);
        //        }
        //        ret->push_back(terms);
    }
    return ret;
}
}  // namespace alica
