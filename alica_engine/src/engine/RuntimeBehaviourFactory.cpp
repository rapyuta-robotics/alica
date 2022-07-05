
#include <engine/RuntimeBehaviourFactory.h>

#include "engine/BasicBehaviour.h"
#include "engine/IAlicaLogger.h"
#include "engine/IBehaviourCreator.h"

#include <alica_common_config/debug_output.h>

namespace alica
{

RuntimeBehaviourFactory::RuntimeBehaviourFactory(std::unique_ptr<IBehaviourCreator>&& bc, IAlicaWorldModel* wm, AlicaEngine* engine, IAlicaLogger& logger)
        : _creator(std::move(bc))
        , _engine(engine)
        , _logger(logger)
        , _wm(wm)
{
}

std::unique_ptr<BasicBehaviour> RuntimeBehaviourFactory::create(int64_t id, const Behaviour* behaviourModel) const
{
    BehaviourContext ctx{_wm, behaviourModel->getName(), behaviourModel};
    std::unique_ptr<BasicBehaviour> basicBeh = _creator->createBehaviour(id, ctx);
    if (!basicBeh) {
        _logger.log(Verbosity::ERROR, "RuntimeBehaviourFactory: Behaviour creation failed: ", id);
        return nullptr;
    }

    // TODO Cleanup: get rid of this later, behaviour only needs traceFactory, teamManager and not entire engine
    basicBeh->setEngine(_engine);
    return basicBeh;
}

} /* namespace alica */
