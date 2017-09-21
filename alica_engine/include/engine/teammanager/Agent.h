#pragma once

#include "engine/IAlicaClock.h"
#include "engine/IRobotID.h"

#include <string>

namespace alica
{

class AlicaEngine;
class RobotProperties;
class RobotEngineData;

class Agent
{
  public:
    Agent(AlicaEngine* engine, AlicaTime timeout, IRobotID *id);
    Agent(AlicaEngine* engine, AlicaTime timeout, IRobotID *id, std::string name);
    virtual ~Agent();

    const IRobotID* getID();
    std::string getName();
    const RobotProperties* getProperties();
    const RobotEngineData* getEngineData();
    bool isActive();
    void setActive(bool active);

  protected:
    IRobotID* id;
    std::string name;
    bool active;
    AlicaTime timeout;
    AlicaTime timeLastMsgReceived;
    RobotProperties* properties;
    RobotEngineData* engineData;

    void setTimeLastMsgReceived(AlicaTime timeLastMsgReceived);

    AlicaEngine* engine;
};

} /* namespace alica */
