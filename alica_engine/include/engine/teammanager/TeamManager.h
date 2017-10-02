#pragma once

#include "engine/ITeamManager.h"
#include "engine/IAlicaClock.h"
#include "engine/teammanager/Agent.h"

#include <map>
#include <list>
#include <memory>
#include <unordered_set>

namespace supplementary {
	class SystemConfig;
	class IAgentID;
}

namespace alica
{

using std::list;
using std::map;
using std::unique_ptr;
using std::unordered_set;

class AlicaEngine;
class Variable;
class SuccessMarks;

class TeamManager : public ITeamManager
{
  public:
	TeamManager(const AlicaEngine * engine, bool useConfigForTeam);
    virtual ~TeamManager();

    void init();

    std::unique_ptr<std::list<Agent*>> getAllAgents();
    std::unique_ptr<std::list<Agent*>> getActiveAgents();
    const supplementary::IAgentID * getLocalAgentID() const;
    std::unique_ptr<std::list<const supplementary::IAgentID *>> getActiveAgentIDs() const;
    std::unique_ptr<std::list<const RobotProperties*>> getActiveAgentProperties() const;
    int getTeamSize() const;
    const Agent* getAgentByID(const supplementary::IAgentID* agentId) const;
    void setTimeLastMsgReceived(const supplementary::IAgentID *agendId, AlicaTime timeLastMsgReceived);
    bool isAgentIgnored(const supplementary::IAgentID* agentId) const;
    bool isAgentActive(const supplementary::IAgentID* agentId) const;
    void ignoreAgent(const supplementary::IAgentID *agentId);
    void unIgnoreAgent(const supplementary::IAgentID *agentId);
    bool setSuccess(const supplementary::IAgentID* agentId, AbstractPlan* plan, EntryPoint* entryPoint);
    bool setSuccessMarks(const supplementary::IAgentID *agentId, std::shared_ptr<SuccessMarks> successMarks);
    Variable* getDomainVariable(const supplementary::IAgentID* robot, std::string sort);

  private:
    AlicaTime teamTimeOut;
    Agent* localAgent;
    map<const supplementary::IAgentID*, Agent*> agents;
    unordered_set<const supplementary::IAgentID*> ignoredAgents;
    bool useConfigForTeam;

    void readTeamFromConfig(supplementary::SystemConfig* sc);
};

} /* namespace alica */
