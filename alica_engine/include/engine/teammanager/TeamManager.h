#pragma once

#include "engine/IAlicaClock.h"
#include "engine/teammanager/Agent.h"

#include <supplementary/AgentID.h>

#include <map>
#include <list>
#include <memory>
#include <unordered_set>

namespace supplementary {
class SystemConfig;
}

namespace alica {

using std::list;
using std::map;
using std::unique_ptr;
using std::unordered_set;

class AlicaEngine;
class Variable;
class SuccessMarks;

class TeamManager {
public:
    TeamManager(AlicaEngine* engine, bool useConfigForTeam);
    virtual ~TeamManager();

    void init();

    std::unique_ptr<std::list<Agent*>> getAllAgents();
    std::unique_ptr<std::list<Agent*>> getActiveAgents();
    const supplementary::AgentID* getLocalAgentID() const;
    const Agent* getLocalAgent() const {return localAgent;}
    void fillWithActiveAgentIDs(std::vector<const supplementary::AgentID*>& oIds) const;
    std::unique_ptr<std::list<const RobotProperties*>> getActiveAgentProperties() const;
    int getTeamSize() const;
    const Agent* getAgentByID(const supplementary::AgentID* agentId) const;
    void setTimeLastMsgReceived(const supplementary::AgentID* agendId, AlicaTime timeLastMsgReceived);
    bool isAgentIgnored(const supplementary::AgentID* agentId) const;
    bool isAgentActive(const supplementary::AgentID* agentId) const;
    void setAgentIgnored(const supplementary::AgentID*, bool) const;
    bool setSuccess(const supplementary::AgentID* agentId, const AbstractPlan* plan, const EntryPoint* entryPoint);
    bool setSuccessMarks(const supplementary::AgentID* agentId, std::shared_ptr<SuccessMarks> successMarks);
    Variable* getDomainVariable(const supplementary::AgentID* robot, std::string sort);
    
private:
    AlicaTime teamTimeOut;
    Agent* localAgent;
    AlicaEngine* engine;
    std::map<const supplementary::AgentID*, Agent*, supplementary::AgentIDComparator> agents;
    bool useConfigForTeam;

    void readTeamFromConfig(supplementary::SystemConfig* sc);
};

} /* namespace alica */
