#pragma once

#include "engine/IAlicaClock.h"

#include <memory>
#include <list>

namespace alica
{

class IRobotID;
class AlicaEngine;
class Agent;

class ITeamManager
{
  public:
    ITeamManager(const AlicaEngine *engine)
        : engine(engine){};
    virtual ~ITeamManager();

    virtual void init() = 0;
    virtual const IRobotID *getLocalAgentID() const = 0;
    virtual std::unique_ptr<std::list<const IRobotID *>> getActiveAgentIDs() const = 0;
    virtual const Agent* getAgentByID(const IRobotID* agentId) const = 0;
    virtual void setTimeLastMsgReceived(const IRobotID *, AlicaTime) = 0;

    const AlicaEngine *engine;
};
}
