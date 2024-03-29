#include "engine/teammanager/Agent.h"

#include "engine/collections/RobotEngineData.h"
#include "engine/collections/RobotProperties.h"
#include "engine/collections/SuccessMarks.h"
#include "engine/containers/AgentAnnouncement.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/EntryPoint.h"

namespace alica
{

Agent::Agent(const PlanRepository& planRepository, const AlicaClock& clock, AlicaTime timeout, const std::string& defaultRole, const AgentAnnouncement& aa)
        : _id(aa.senderID)
        , _token(aa.token)
        , _name(aa.senderName)
        , _planHash(aa.planHash)
        , _sdk(aa.senderSdk)
        , _clock(clock)
        , _properties(defaultRole, aa)
        , _engineData(planRepository, aa.senderID)
        , _timeout(timeout)
        , _active(false)
        , _ignored(false)
        , _local(false)
{
}

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

void Agent::setSuccess(const AbstractPlan* plan, const EntryPoint* entryPoint)
{
    _engineData.editSuccessMarks().markSuccessful(plan, entryPoint);
}

void Agent::setSuccessMarks(const IdGrp& succeededEps)
{
    _engineData.updateSuccessMarks(succeededEps);
}

const DomainVariable* Agent::getDomainVariable(const std::string& sort) const
{
    return _engineData.getDomainVariable(sort);
}

const EntryPointGrp* Agent::getSucceededEntryPoints(const AbstractPlan* plan) const
{
    return _engineData.getSuccessMarks().succeededEntryPoints(plan);
}

bool Agent::update()
{
    if (_local) {
        return false;
    }
    const auto time = _timeLastMsgReceived.load() + _timeout;
    if (_active && time < _clock.now()) {
        // timeout triggered
        _engineData.clearSuccessMarks();
        _active = false;
        return true;
    }

    if (!_active && time > _clock.now()) {
        // reactivate because of new messages
        _active = true;
        return true;
    }

    return false;
}

} /* namespace alica */
