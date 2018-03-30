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
    const Agent* getLocalAgent() const {
        return localAgent;
    }
    std::unique_ptr<std::list<const supplementary::AgentID*>> getActiveAgentIDs() const;
    std::unique_ptr<std::list<const RobotProperties*>> getActiveAgentProperties() const;
    int getTeamSize() const;
    const Agent* getAgentByID(const supplementary::AgentID* agentId) const;
    void setTimeLastMsgReceived(const supplementary::AgentID* agendId, AlicaTime timeLastMsgReceived);
    bool isAgentIgnored(const supplementary::AgentID* agentId) const;
    bool isAgentActive(const supplementary::AgentID* agentId) const;
    void ignoreAgent(const supplementary::AgentID* agentId);
    void unIgnoreAgent(const supplementary::AgentID* agentId);
    bool setSuccess(const supplementary::AgentID* agentId, AbstractPlan* plan, EntryPoint* entryPoint);
    bool setSuccessMarks(const supplementary::AgentID* agentId, std::shared_ptr<SuccessMarks> successMarks);
    Variable* getDomainVariable(const supplementary::AgentID* robot, std::string sort);

private:
    AlicaTime teamTimeOut;
    Agent* localAgent;
    AlicaEngine* engine;
    std::map<const supplementary::AgentID*, Agent*, supplementary::AgentIDComparator> agents;
    std::unordered_set<const supplementary::AgentID*, supplementary::AgentIDHash,
            supplementary::AgentIDEqualsComparator>
            ignoredAgents;
    bool useConfigForTeam;

    void readTeamFromConfig(supplementary::SystemConfig* sc);
};

} /* namespace alica */
