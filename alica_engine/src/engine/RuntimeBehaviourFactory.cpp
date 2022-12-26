
#include <engine/RuntimeBehaviourFactory.h>

#include "engine/BasicBehaviour.h"
#include "engine/IBehaviourCreator.h"
#include "engine/logging/Logging.h"
#include "engine/modelmanagement/factories/Factory.h"

namespace alica
{

RuntimeBehaviourFactory::RuntimeBehaviourFactory(Blackboard& globalBlackboard, TeamManager& teamManager, PlanBase& planBase,
        const IAlicaCommunication& communication, const IAlicaTraceFactory* traceFactory, const IAlicaTimerFactory& timerFactory)
        : _globalBlackboard(globalBlackboard)
        , _teamManager(teamManager)
        , _planBase(planBase)
        , _communication(communication)
        , _traceFactory(traceFactory)
        , _timerFactory(timerFactory)
{
}

void RuntimeBehaviourFactory::init(std::unique_ptr<IBehaviourCreator>&& bc)
{
    _creator = std::move(bc);
}

std::unique_ptr<BasicBehaviour> RuntimeBehaviourFactory::create(int64_t id, const Behaviour* behaviourModel) const
{
    BehaviourContext ctx{_globalBlackboard, behaviourModel->getName(), behaviourModel, _traceFactory};
    std::unique_ptr<BasicBehaviour> basicBeh = _creator->createBehaviour(id, ctx);
    if (!basicBeh) {
        Logging::logError("RuntimeBehaviourFactory") << "Behaviour creation failed: " << id;
        return nullptr;
    }

    basicBeh->setTeamManager(&_teamManager);
    basicBeh->setPlanBase(&_planBase);
    basicBeh->setAlicaCommunication(&_communication);
    basicBeh->setAlicaTraceFactory(_traceFactory);
    basicBeh->setAlicaTimerFactory(&_timerFactory);
    return basicBeh;
}

} /* namespace alica */
