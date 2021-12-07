#include "engine/teammanager/Agent.h"

#include "engine/AlicaEngine.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/collections/RobotProperties.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/containers/AgentAnnouncement.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"

namespace alica
{

Agent::Agent(const AlicaEngine* engine, AlicaTime timeout, const std::string& defaultRole, const AgentAnnouncement& aa)
        : _id(aa.senderID)
        , _token(aa.token)
        , _name(aa.senderName)
        , _planHash(aa.planHash)
        , _sdk(aa.senderSdk)
        , _engine(engine)
        , _properties(engine, defaultRole, aa)
        , _engineData(engine, aa.senderID)
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

void Agent::setTimeout(AlicaTime t)
{
    _timeout = t;
}

void Agent::setSuccess(std::size_t parentContextHash, const EntryPoint* entryPoint)
{
    _engineData.editSuccessMarks().markSuccessful(parentContextHash, entryPoint);
}

void Agent::setSuccessMarks(const std::vector<std::size_t>& suceededContexts)
{
    _engineData.updateSuccessMarks(suceededContexts);
}

const DomainVariable* Agent::getDomainVariable(const std::string& sort) const
{
    return _engineData.getDomainVariable(sort);
}

EntryPointGrp Agent::getSucceededEntryPoints(std::size_t parentContextHash, const AbstractPlan* plan) const
{
    return _engineData.getSuccessMarks().succeededEntryPoints(parentContextHash, plan->getId());
}

bool Agent::update()
{
    if (_local) {
        return false;
    }
    if (_active && _timeLastMsgReceived + _timeout < _engine->getAlicaClock().now()) {
        // timeout triggered
        _engineData.clearSuccessMarks();
        _active = false;
        return true;
    }

    if (!_active && _timeLastMsgReceived + _timeout > _engine->getAlicaClock().now()) {
        // reactivate because of new messages
        _active = true;
        return true;
    }

    return false;
}

} /* namespace alica */
