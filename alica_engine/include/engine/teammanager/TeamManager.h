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

class TeamManager : public ITeamManager
{
  public:
	TeamManager(const AlicaEngine * engine, bool useConfigForTeam);
    virtual ~TeamManager();

    void init();

    const IRobotID * getLocalAgentID() const;
    std::unique_ptr<std::list<const IRobotID *>> getActiveAgentIDs() const;
    const Agent* getAgentByID(const IRobotID* agentId) const;
    void setTimeLastMsgReceived(const IRobotID *agendId, AlicaTime timeLastMsgReceived);
    bool isAgentIgnored(const IRobotID* agentId) const;

  private:
    AlicaTime teamTimeOut;
    Agent * localAgent;
    map<const IRobotID *, Agent *> agents;
    unordered_set<const alica::IRobotID *> ignoredAgents;
    bool useConfigForTeam;

    void readTeamFromConfig(supplementary::SystemConfig *sc);
};

} /* namespace alica */
