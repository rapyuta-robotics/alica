#pragma once

#include "engine/IAlicaClock.h"

#include <memory>
#include <vector>

namespace alica
{

class IRobotID;
class AlicaEngine;

class ITeamManager
{
  public:
    ITeamManager(const AlicaEngine *engine)
        : engine(engine){};
    virtual ~ITeamManager();

    virtual void init() = 0;
    virtual void tick() = 0;
    virtual const IRobotID *getOwnRobotID() const = 0;
    virtual std::unique_ptr<std::vector<const IRobotID *>> getActiveRobots() const = 0;
    virtual void setTimeLastMsgReceived(const IRobotID *, AlicaTime) = 0;

    const AlicaEngine *engine;
};
}
