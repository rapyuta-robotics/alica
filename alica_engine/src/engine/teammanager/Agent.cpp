#include "engine/teammanager/Agent.h"

#include "engine/AlicaEngine.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/RobotProperties.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"
#include "supplementary/IAgentID.h"

namespace alica
{

Agent::Agent(const AlicaEngine *engine, AlicaTime timeout, const supplementary::IAgentID *id)
    : id(id)
    , name("")
    , timeLastMsgReceived(0)
    , engine(engine)
    , properties(nullptr)
    , engineData(nullptr)
    , timeout(timeout)
    , active(false)
    , local(false)
{
}

Agent::Agent(const AlicaEngine *engine, AlicaTime timeout, const supplementary::IAgentID *id, std::string name)
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

const supplementary::IAgentID *Agent::getID()
{
    return this->id;
}

std::string Agent::getName()
{
    return this->name;
}

bool Agent::isActive()
{
    return this->active;
}

void Agent::setLocal(bool local)
{
    this->local = local;
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

void Agent::setSuccess(AbstractPlan *plan, EntryPoint *entryPoint)
{
    this->engineData->getSuccessMarks()->markSuccessfull(plan, entryPoint);
}

void Agent::setSuccessMarks(std::shared_ptr<SuccessMarks> successMarks)
{
    this->engineData->setSuccessMarks(successMarks);
}

Variable *Agent::getDomainVariable(std::string sort)
{
    return this->engineData->getDomainVariable(sort);
}

std::shared_ptr<std::list<EntryPoint *>> Agent::getSucceededEntryPoints(AbstractPlan *plan)
{
    return this->engineData->getSuccessMarks()->succeededEntryPoints(plan);
}

bool Agent::update()
{
    if (this->active && this->timeLastMsgReceived + this->timeout < this->engine->getIAlicaClock()->now())
    {
        // timeout triggered
        this->engineData->clearSuccessMarks();
        this->active = false;
        return true;
    }

    if (!this->active && this->timeLastMsgReceived + this->timeout > this->engine->getIAlicaClock()->now())
    {
        // reactivate because of new messages
        this->active = true;
        return true;
    }

    return false;
}

} /* namespace alica */
