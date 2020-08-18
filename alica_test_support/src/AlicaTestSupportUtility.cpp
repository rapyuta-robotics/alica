#include "alica/AlicaTestSupportUtility.h"

#include "alica/AlicaTestBehaviourCreator.h"

#include <engine/BasicBehaviour.h>
#include <engine/model/Behaviour.h>

namespace alica
{

alica::AlicaEngine* AlicaTestSupportUtility::getEngine(alica::AlicaContext* ac)
{
    return ac->_engine.get();
}

std::pair<bool, std::string> AlicaTestSupportUtility::getStateName(alica::AlicaContext* ac, uint64_t stateID)
{
    std::pair<bool, std::string> returnPair(false, "");
    const State* state = ac->_engine->getPlanRepository().getStates().find(stateID);
    if (state != nullptr) {
        returnPair.first = true;
        returnPair.second = state->getName();
    }
    return returnPair;
}

std::unique_ptr<alica::AlicaTestBehaviourTrigger> AlicaTestSupportUtility::makeBehaviourTriggered(
        alica::AlicaContext* ac, AlicaTestBehaviourCreator* testBehaviourCreator, int64_t behaviourID)
{
    // make behaviour event driven
    const Behaviour* constBehaviour = ac->_engine->getPlanRepository().getBehaviours().find(behaviourID);
    if (constBehaviour == nullptr) {
        return nullptr;
    }
    Behaviour* behaviour = const_cast<Behaviour*>(constBehaviour);
    behaviour->setEventDriven(true);

    // create basic behaviour
    std::shared_ptr<BasicBehaviour> basicBehaviour = testBehaviourCreator->createBehaviour(behaviourID);

    // set trigger
    std::unique_ptr<alica::AlicaTestBehaviourTrigger> trigger = std::make_unique<alica::AlicaTestBehaviourTrigger>();
    basicBehaviour->setTrigger(trigger.get());

    // move ownership of trigger
    return std::move(trigger);
}
} // namespace alica
