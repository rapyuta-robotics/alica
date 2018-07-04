#pragma once

#include <engine/AgentIDConstPtr.h>
#include <engine/AlicaClock.h>
#include <engine/teammanager/Agent.h>

#include <list>
#include <map>
#include <memory>
#include <string>
#include <unordered_set>

namespace supplementary
{
class SystemConfig;
}

namespace alica
{

class AlicaEngine;
class DomainVariable;
class Variable;
class SuccessMarks;

class TeamManager
{
public:
    TeamManager(AlicaEngine* engine, bool useConfigForTeam);
    virtual ~TeamManager();

    void init();

    std::unique_ptr<std::list<Agent*>> getAllAgents();
    std::unique_ptr<std::list<Agent*>> getActiveAgents();
    AgentIDConstPtr getLocalAgentID() const;
    const Agent* getLocalAgent() const { return localAgent; }
    void fillWithActiveAgentIDs(std::vector<AgentIDConstPtr>& oIds) const;
    std::unique_ptr<std::list<const RobotProperties*>> getActiveAgentProperties() const;
    int getTeamSize() const;
    const Agent* getAgentByID(AgentIDConstPtr agentId) const;
    void setTimeLastMsgReceived(AgentIDConstPtr agendId, AlicaTime timeLastMsgReceived);
    bool isAgentIgnored(AgentIDConstPtr agentId) const;
    bool isAgentActive(AgentIDConstPtr agentId) const;
    void setAgentIgnored(AgentIDConstPtr, bool) const;
    bool setSuccess(AgentIDConstPtr agentId, const AbstractPlan* plan, const EntryPoint* entryPoint);
    bool setSuccessMarks(AgentIDConstPtr agentId, std::shared_ptr<SuccessMarks> successMarks);
    const DomainVariable* getDomainVariable(AgentIDConstPtr robot, const std::string& sort) const;

private:
    AlicaTime teamTimeOut;
    Agent* localAgent;
    AlicaEngine* engine;
    std::map<AgentIDConstPtr, Agent*> agents;
    bool useConfigForTeam;

    void readTeamFromConfig(supplementary::SystemConfig* sc);
};

} /* namespace alica */
