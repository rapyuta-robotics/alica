#include "engine/teammanager/Agent.h"

#include "engine/AgentIDConstPtr.h"
#include "engine/AlicaEngine.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/RobotProperties.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"

namespace alica
{

Agent::Agent(const AlicaEngine* engine, AlicaTime timeout, AgentIDConstPtr id)
        : _id(id)
        , _name()
        , _engine(engine)
        , _properties()
        , _engineData(engine, id)
        , _timeout(timeout)
        , _active(false)
        , _ignored(false)
        , _local(false)
{
}

Agent::Agent(const AlicaEngine* engine, AlicaTime timeout, AgentIDConstPtr id, const std::string& name)
        : _id(id)
        , _name(name)
        , _engine(engine)
        , _properties(engine, name)
        , _engineData(engine, id)
        , _timeout(timeout)
        , _active(false)
        , _ignored(false)
        , _local(false)
{
}

Agent::~Agent() {}

void Agent::setLocal(bool local)
{
    if (local) {
        _active = true;
    }
    _local = local;
}

void Agent::setSuccess(const AbstractPlan* plan, const EntryPoint* entryPoint)
{
    _engineData.getSuccessMarks()->markSuccessfull(plan, entryPoint);
}

void Agent::setSuccessMarks(std::shared_ptr<SuccessMarks> successMarks)
{
    _engineData.setSuccessMarks(successMarks);
}

const DomainVariable* Agent::getDomainVariable(const std::string& sort) const
{
    return _engineData.getDomainVariable(sort);
}

const EntryPointGrp* Agent::getSucceededEntryPoints(const AbstractPlan* plan) const
{
    return _engineData.getSuccessMarks()->succeededEntryPoints(plan);
}

bool Agent::update()
{
    if (_local) {
        return false;
    }
    if (_active && _timeLastMsgReceived + _timeout < _engine->getAlicaClock()->now()) {
        // timeout triggered
        _engineData.clearSuccessMarks();
        _active = false;
        return true;
    }

    if (!_active && _timeLastMsgReceived + _timeout > _engine->getAlicaClock()->now()) {
        // reactivate because of new messages
        _active = true;
        return true;
    }

    return false;
}

} /* namespace alica */
