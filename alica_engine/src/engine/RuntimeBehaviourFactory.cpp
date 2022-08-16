
#include <engine/RuntimeBehaviourFactory.h>

#include "engine/BasicBehaviour.h"
#include "engine/IBehaviourCreator.h"
#include "engine/logging/Logging.h"

namespace alica
{

RuntimeBehaviourFactory::RuntimeBehaviourFactory(IAlicaWorldModel* wm, AlicaEngine* engine)
        : _engine(engine)
        , _wm(wm)
{
}

void RuntimeBehaviourFactory::init(std::unique_ptr<IBehaviourCreator>&& bc)
{
    _creator = std::move(bc);
}

std::unique_ptr<BasicBehaviour> RuntimeBehaviourFactory::create(int64_t id, const Behaviour* behaviourModel) const
{
    BehaviourContext ctx{_wm, behaviourModel->getName(), behaviourModel};
    std::unique_ptr<BasicBehaviour> basicBeh = _creator->createBehaviour(id, ctx);
    if (!basicBeh) {
        Logging::logError("RuntimeBehaviourFactory") << "Behaviour creation failed: " << id;
        return nullptr;
    }

    // TODO Cleanup: get rid of this later, behaviour only needs traceFactory, teamManager and not entire engine
    basicBeh->setEngine(_engine);
    return basicBeh;
}

} /* namespace alica */
