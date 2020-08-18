#include "alica/AlicaTestSupportUtility.h"

#include <engine/AlicaContext.h>
#include <engine/AlicaEngine.h>

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

bool AlicaTestSupportUtility::stepUntilStateReached(alica::AlicaContext* ac, int64_t state, uint64_t msTimeout)
{
    std::chrono::milliseconds timeout(msTimeout);
    auto start = std::chrono::system_clock::now();
    while (std::chrono::system_clock::now() - start < timeout) {
        ac->stepEngine();
        if (ac->isStateActive(state)) {
            return true;
        }
    }
    std::cout << "Stuck in state " << ac->_engine->getPlanBase().getDeepestNode()->getActiveState()->getName() << std::endl;
    return false;
}

} // namespace alica
