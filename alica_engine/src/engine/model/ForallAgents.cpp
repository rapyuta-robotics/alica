#include "engine/model/ForallAgents.h"
#include "engine/AlicaEngine.h"
#include "engine/Assignment.h"
#include "engine/RunningPlan.h"
#include "engine/TeamObserver.h"
#include "engine/collections/AgentVariables.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/DomainVariable.h"
#include "engine/model/Plan.h"
#include "engine/model/State.h"
#include "engine/teammanager/Agent.h"
#include "engine/teammanager/TeamManager.h"

#include <assert.h>

namespace alica
{

ForallAgents::ForallAgents(int64_t id)
        : Quantifier(id)
{
}

ForallAgents::~ForallAgents() {}

ForallAgents::Result ForallAgents::TryAddId(AgentIDConstPtr id, std::vector<AgentVariables>& io_agentVarsInScope, const TeamManager* tm) const
{
    std::vector<AgentVariables>::iterator it =
            std::find_if(io_agentVarsInScope.begin(), io_agentVarsInScope.end(), [id](const AgentVariables& av) { return av.getId() == id; });

    const RobotEngineData* robotEngineData = tm->getAgentByID(id)->getEngineData();
    if (it == io_agentVarsInScope.end()) {
        // add new agent
        AgentVariables newAgent(id);
        newAgent.editVars().reserve(getDomainIdentifiers().size());
        std::transform(getDomainIdentifiers().begin(), getDomainIdentifiers().end(), std::back_inserter(newAgent.editVars()),
                [robotEngineData](const std::string& s) -> const DomainVariable* { return robotEngineData->getDomainVariable(s); });
        io_agentVarsInScope.push_back(std::move(newAgent));
        return ADDED;
    } else {
        // modify existing agent
        AgentVariables& oldAgent = *it;
        Result r = NONE;
        for (const std::string& s : getDomainIdentifiers()) {
            if (std::find_if(oldAgent.getVars().begin(), oldAgent.getVars().end(), [&s](const DomainVariable* v) { return v->getName() == s; }) ==
                    oldAgent.getVars().end()) {
                oldAgent.editVars().push_back(robotEngineData->getDomainVariable(s));
                r = MODIFIED;
            }
        }
        return r;
    }
}

bool ForallAgents::isAgentInScope(AgentIDConstPtr id, const RunningPlan& rp) const
{
    switch (getScopeType()) {
    case PLANSCOPE:
        return rp.getActivePlan() == getScopedPlan() && rp.getAssignment().hasAgent(id);
        break;
    case ENTRYPOINTSCOPE: {
        return rp.getAssignment().getEntryPointOfAgent(id) == getScopedEntryPoint();
        break;
    case STATESCOPE:
        return rp.getAssignment().getStateOfAgent(id) == getScopedState();
        break;
    }
        assert(false);
        return false;
    }

    bool ForallAgents::addDomainVariables(const RunningPlan& p, std::vector<AgentVariables>& io_agentVarsInScope) const
    {
        bool addedAgent = false;
        bool changedAgent = false;

        const TeamManager* tm = p.getAlicaEngine()->getTeamManager();
        switch (getScopeType()) {
        case PLANSCOPE:
            if (p->getActivePlan() == getScopedPlan()) {
                for (AgentIDConstPtr id : p.getAssignment().getAllAgents()) {
                    Result r = TryAddId(id, io_agentVarsInScope, tm);
                    addedAgent = addedAgent || r == ADDED;
                    changedAgent = changedAgent || r == MODIFIED;
                }
            }
            break;
        case ENTRYPOINTSCOPE:
            if (p->getActivePlan() == getScopedEntryPoint()->getPlan()) {
                for (AgentIDConstPtr id : *p.getAssignment().getAgentsWorking(getScopedEntryPoint())) {
                    Result r = TryAddId(id, io_agentVarsInScope, tm);
                    addedAgent = addedAgent || r == ADDED;
                    changedAgent = changedAgent || r == MODIFIED;
                }
            }
            break;

        case STATESCOPE:
            if (p->getActivePlan() == getScopedState()->getInPlan()) {
                for (AgentIDConstPtr id : p.getAssignment()->getAgentsInState(getScopedState())) {
                    Result r = TryAddId(id, io_agentVarsInScope, tm);
                    addedAgent = addedAgent || r == ADDED;
                    changedAgent = changedAgent || r == MODIFIED;
                }
            }
            break;
        }

        if (addedAgent) {
            std::sort(io_agentVarsInScope.begin(), io_agentVarsInScope.end());
        }
        return addedAgent || changedAgent;
    }
} // namespace alica
