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

namespace {
enum Result {
    ADDED,MODIFIED,NONE
};
Result TryAddId(AgentIdPtr id, std::vector<AgentVariables>& io_agentVarsInScope, const TeamManager* tm) {

    std::vector<AgentVariables>::iterator it = std::find_if(_io_agentVarsInScope.begin(),_io_agentVarsInScope.end(),
        [id](const AgentVariables& av) {return av.getId() == id;});
    

    const RobotEngineData* robotEngineData = tm->getAgentByID(id)->getEngineData();
    if(it == io_agentVarsInScope.end()) {
        //add new agent
        AgentVariables newAgent(id);
        newAgent.editVars().reserve(getDomainIdentifiers().size());
        std::transform(getDomainIdentifiers().begin(), getDomainIdentifiers().end(),
            std::back_inserter(newAgent.editVars()), [robotEngineData](const std::string& s) -> const Variable* {
                return robotEngineData->getDomainIdentifiers(s);
            });
        io_agentVarsInScope.push_back(std::move(newAgent));
        return ADDED;
    } else {
        //modify existing agent
        AgentVariables& oldAgent = *it;
        Result r = NONE;
        for(const std::string& s : getDomainIdentifiers()) {
            if(std::find_if(oldAgent.getVars().begin(), oldAgent.getVars().end(),[&s](const Variable* v) {return v->getName() == s;}) == oldAgent.getVars().end()) {
                oldAgent.editVars().push_back(robotEngineData->getDomainIdentifiers(s));
                r = MODIFIED;
            }
        }
        return r;
    }
}
}

virtual bool ForallAgents::isAgentUnderScope(AgentIdPtr id, std::shared_ptr<RunningPlan>& rp) const {
    switch(getScopeType()) {
        case PLANSCOPE:
        return rp->getPlan() == getScopedPlan() && std::find(rp.getRobotsAvail().begin(),rp.getRobotsAvail().end(),id) != rp.getRobotsAvail().end();
        break;
        case ENTRYPOINTSCOPE:
        const AgentSet* agents = rp->getAssignment()->getRobotsWorking(getScopedEntryPoint());
        return std::find(agents.begin(), agents.end(), id) != agents.end();
        break;
        case STATESCOPE:
        return p->getAssignment()->getRobotStateMapping()->getStateOfRobot() == getScopedState();
        break;
    }

}

bool ForallAgents::addDomainVariables(std::shared_ptr<RunningPlan>& p, std::vector<AgentVariables>& io_agentVarsInScope) const {
    bool addedAgent = false;
    bool changedAgent = false;
    
    const TeamManager* tm = p->getAlicaEngine()->getTeamManager();
    switch(getScopeType()) {
        case PLANSCOPE:
        if (p->getPlan() == getScopedPlan()) {
            for(AgentIdPtr id : p->getRobotsAvail()) {
                Result r = TryAddId(id,io_agentVarsInScope,tm);
                addedAgent = addedAgent || r == ADDED;
                changedAgent = changedAgent || r == MODIFIED;
            }
        }
        break;
        case ENTRYPOINTSCOPE:
        for(AgentIdPtr id : *p->getAssignment()->getRobotsWorking(getScopedEntryPoint()) {
                Result r = TryAddId(id,io_agentVarsInScope,tm);
                addedAgent = addedAgent || r == ADDED;
                changedAgent = changedAgent || r == MODIFIED;
        }
        break;
        case STATESCOPE:
    
        for(AgentIdPtr id : p->getAssignment()->getRobotStateMapping()->getRobotsInState(getScopedState()) {
                Result r = TryAddId(id,io_agentVarsInScope,tm);
                addedAgent = addedAgent || r == ADDED;
                changedAgent = changedAgent || r == MODIFIED;
        }
        break;
    }
    

    if(addedAgent) {
        std::sort(io_agentVarsInScope.begin(),io_agentVarsInScope.end());
    }
    return addedAgent || changedAgent;
}
}  // namespace alica
