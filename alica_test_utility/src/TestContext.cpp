#include "alica/test/TestContext.h"

#include "alica/test/TestBehaviourCreator.h"

#include <engine/BasicBehaviour.h>
#include <engine/model/Behaviour.h>
#include <engine/model/ConfAbstractPlanWrapper.h>
#include <engine/model/Configuration.h>

namespace alica::test
{

TestContext::TestContext(const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine, const essentials::IdentifierConstPtr agentID)
        : AlicaContext(roleSetName, masterPlanName, stepEngine, agentID)
{
}

alica::AlicaEngine* TestContext::getEngine()
{
    return _engine.get();
}

bool TestContext::makeBehaviourEventDriven(int64_t behaviourID)
{
    const Behaviour* constBehaviour = _engine->getPlanRepository().getBehaviours().find(behaviourID);
    if (constBehaviour == nullptr) {
        return false;
    }
    const_cast<Behaviour*>(constBehaviour)->setEventDriven(true);
    return true;
}

std::unique_ptr<BehaviourTrigger> TestContext::setBehaviourTrigger(int64_t behaviourID, int64_t configurationID)
{
    std::shared_ptr<alica::BasicBehaviour> behaviour = nullptr;
    for (auto& behaviourEntry : _engine->getBehaviourPool().getAvailableBehaviours()) {
        if (behaviourEntry.first->getAbstractPlan()->getId() == behaviourID &&
                (configurationID == 0 ? behaviourEntry.first->getConfiguration() == nullptr
                                      : behaviourEntry.first->getConfiguration()->getId() == configurationID)) {
            behaviour = behaviourEntry.second;
        }
    }

    if (behaviour) {
        std::unique_ptr<BehaviourTrigger> trigger = std::make_unique<BehaviourTrigger>();
        behaviour->setTrigger(trigger.get());
        return std::move(trigger);
    } else {
        return nullptr;
    }
}

} // namespace alica::test
