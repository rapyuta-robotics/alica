
#include <engine/RuntimeBehaviourFactory.h>

#include "engine/IBehaviourCreator.h"
#include "engine/BasicBehaviour.h"

#include <alica_common_config/debug_output.h>

namespace alica
{

RuntimeBehaviourFactory::RuntimeBehaviourFactory(IBehaviourCreator& bc, const PlanRepository::MapType<Behaviour>& behaviourRepo, IAlicaWorldModel *wm, AlicaEngine *engine)
    : _creator(bc)
    , _behaviourRepo(behaviourRepo)
    , _engine(engine) {}

std::unique_ptr<BasicBehaviour> RuntimeBehaviourFactory::create(int64_t id) const
{
    const auto it = _behaviourRepo.find(id);
    if(it == _behaviourRepo.end()) {
        ALICA_ERROR_MSG("RuntimeBehaviourFactory: Behaviour not found in repo: " << id);
        return nullptr;
    }
    const auto* behaviour = it->second;

    BehaviourContext ctx{_wm, behaviour->getName(), behaviour};
    std::unique_ptr<BasicBehaviour> basicBeh = _creator.createBehaviour(id, ctx);
    if (!basicBeh) {
        ALICA_ERROR_MSG("RuntimeBehaviourFactory: Behaviour creation failed: " << id);
        return nullptr;
    }

    // TODO Cleanup: get rid of this later, behaviour only needs traceFactory, teamManager and not entire engine
    basicBeh->setEngine(_engine);
    return basicBeh;
}

} /* namespace alica */
