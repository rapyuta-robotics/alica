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
}

namespace alica
{

using std::list;
using std::map;
using std::unique_ptr;
using std::unordered_set;

class IRobotID;
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
    const IRobotID * getLocalAgentID() const;
    std::unique_ptr<std::list<const IRobotID *>> getActiveAgentIDs() const;
    std::unique_ptr<std::list<const RobotProperties*>> getActiveAgentProperties() const;
    int getTeamSize() const;
    const Agent* getAgentByID(const IRobotID* agentId) const;
    void setTimeLastMsgReceived(const IRobotID *agendId, AlicaTime timeLastMsgReceived);
    bool isAgentIgnored(const IRobotID* agentId) const;
    bool isAgentActive(const IRobotID* agentId) const;
    void ignoreAgent(const alica::IRobotID *agentId);
    void unIgnoreAgent(const alica::IRobotID *agentId);
    bool setSuccess(const IRobotID* agentId, AbstractPlan* plan, EntryPoint* entryPoint);
    bool setSuccessMarks(const IRobotID *agentId, std::shared_ptr<SuccessMarks> successMarks);
    Variable* getDomainVariable(const IRobotID* robot, std::string sort);

  private:
    AlicaTime teamTimeOut;
    Agent* localAgent;
    map<const IRobotID*, Agent*> agents;
    unordered_set<const alica::IRobotID*> ignoredAgents;
    bool useConfigForTeam;

    void readTeamFromConfig(supplementary::SystemConfig* sc);
};

} /* namespace alica */
