#pragma once

#include "engine/IAlicaClock.h"

#include <string>

namespace alica
{

class AlicaEngine;
class RobotProperties;
class RobotEngineData;
class IRobotID;
class TeamManager;

class Agent
{
    // allows the TeamManager to call setTimeLastMsgReceived(..)
    friend ::alica::TeamManager;

  public:
    virtual ~Agent();

    const IRobotID *getID();
    std::string getName();
    const RobotProperties *getProperties() const;
    const RobotEngineData *getEngineData() const;
    bool isActive();
    void setActive(bool active);

  protected:
    Agent(const AlicaEngine *engine, AlicaTime timeout, const IRobotID *id);
    Agent(const AlicaEngine *engine, AlicaTime timeout, const IRobotID *id, std::string name);

    const IRobotID *id;
    std::string name;
    bool active;
    AlicaTime timeout;
    AlicaTime timeLastMsgReceived;
    RobotProperties *properties;
    RobotEngineData *engineData;

    void setTimeLastMsgReceived(AlicaTime timeLastMsgReceived);

    const AlicaEngine *engine;
};

} /* namespace alica */
