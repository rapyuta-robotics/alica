#include "engine/teammanager/Agent.h"

#include "engine/IRobotID.h"
#include "engine/AlicaEngine.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/RobotProperties.h"

namespace alica
{

Agent::Agent(const AlicaEngine *engine, AlicaTime timeout, const IRobotID *id)
    : id(id)
    , name("")
    , timeLastMsgReceived(0)
    , engine(engine)
    , properties(nullptr)
    , engineData(nullptr)
    , timeout(timeout)
    , active(false)
{
}

Agent::Agent(const AlicaEngine *engine, AlicaTime timeout, const IRobotID *id, std::string name)
    : Agent(engine, timeout, id)
{
    this->name = name;
    this->properties = new RobotProperties(id);
    this->engineData = new RobotEngineData(engine, id);
}

Agent::~Agent()
{
    delete this->properties;
}

const IRobotID *Agent::getID()
{
    return this->id;
}

std::string Agent::getName()
{
    return this->name;
}

bool Agent::isActive()
{
    AlicaTime now = this->engine->getIAlicaClock()->now();
    return this->timeLastMsgReceived + timeout < now;
}

void Agent::setActive(bool active)
{
    this->active = active;
}

const RobotProperties *Agent::getProperties() const
{
    return this->properties;
}

const RobotEngineData *Agent::getEngineData() const
{
    return this->engineData;
}

void Agent::setTimeLastMsgReceived(AlicaTime timeLastMsgReceived)
{
    this->timeLastMsgReceived = timeLastMsgReceived;
}

} /* namespace alica */
