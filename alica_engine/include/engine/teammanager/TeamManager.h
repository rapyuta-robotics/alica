#pragma once

#include "engine/ITeamManager.h"
#include "engine/IAlicaClock.h"

#include <map>
#include <vector>
#include <memory>
#include <unordered_set>

namespace supplementary {
	class SystemConfig;
}

namespace alica
{

using std::vector;
using std::map;
using std::unique_ptr;
using std::unordered_set;

class IRobotID;
class AlicaEngine;
class Agent;

class TeamManager : public ITeamManager
{
  public:
	TeamManager(const AlicaEngine * engine, bool useConfigForTeam);
    virtual ~TeamManager();

    void init();
    void tick();

    const IRobotID * getLocalAgentID() const;
    void setTimeLastMsgReceived(const IRobotID *robotID, AlicaTime timeLastMsgReceived);

  private:
    AlicaTime teamTimeOut;
    Agent * localAgent;
    map<const IRobotID *, Agent *> agents;
    unordered_set<const alica::IRobotID *> ignoredRobots;
    bool useConfigForTeam;

    void readTeamFromConfig(supplementary::SystemConfig *sc);
};

} /* namespace alica */
