#pragma once

#include "alica/AlicaTestBehaviourTrigger.h"

#include <engine/AlicaContext.h>
#include <engine/AlicaEngine.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace alica
{
class AlicaEngine;
class AlicaContext;
class AlicaTestBehaviourCreator;
class AlicaTestBehaviourTrigger;
class AlicaTestSupportUtility
{
public:
    AlicaTestSupportUtility() = delete;
    /**
     * This allows to get hold of the AlicaEngine, which is
     * an inaccessible member variable of the AlicaContext, otherwise.
     * @param ac The AlicaContext, which owns the requested AlicaEngine.
     * @return Pointer to the AlicaEngine.
     */
    static alica::AlicaEngine* getEngine(alica::AlicaContext* ac);

    /**
     * Allows to retrieve the name of a state.
     * @param ac The AlicaContext which should own the state, whose name is requested.
     * @param stateID The ID of the state, whose name is requested.
     * @return A pair of bool and string. The boolean indicate whether the requested
     * State is part of the given AlicaContext, or not. If true, the string contains
     * the name of the State.
     */
    static std::pair<bool, std::string> getStateName(alica::AlicaContext* ac, uint64_t stateID);

    /**
     * This method requests the engine to step until the given state is reached, or the
     * time is over.
     * @param ac The AlicaContext for which step is called.
     * @param state The state that the AlicaContext should reach before the timeout.
     * @param msTimeout The timeout in milliseconds
     * @return True, if the given state was reached before the timeout. False, otherwise.
     */
    template <typename Rep, typename Period>
    static bool stepUntilStateReached(alica::AlicaContext* ac, int64_t state, std::chrono::duration<Rep, Period> timeout);

    template <typename Rep, typename Period>
    static bool stepBehaviour(alica::AlicaTestBehaviourTrigger* behaviourTrigger, std::chrono::duration<Rep, Period> timeout);

    /**
     * IMPORTANT: This method must be called, before the AlicaContext is initialised via its init-Method!
     *
     * This method changes the eventDriven property of the referenced behaviour to be true, set an
     * EventTrigger object for the behaviour and returns the EventTrigger object to the caller.
     * @param ac The AlicaContext that includes the given behaviour.
     * @param behaviourID The ID of the behaviour that should be changed to execute in an event driven fashion.
     * @return The EventTrigger object which can be used to trigger the behaviour, if
     * the behaviour existed. Nullptr, otherwise.
     */
    static std::unique_ptr<alica::AlicaTestBehaviourTrigger> makeBehaviourTriggered(
            alica::AlicaContext* ac, AlicaTestBehaviourCreator* testBehaviourCreator, int64_t behaviourID);
};

template <typename Rep, typename Period>
bool AlicaTestSupportUtility::stepUntilStateReached(alica::AlicaContext* ac, int64_t state, std::chrono::duration<Rep, Period> timeout)
{
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

template <typename Rep, typename Period>
bool AlicaTestSupportUtility::stepBehaviour(alica::AlicaTestBehaviourTrigger* behaviourTrigger, std::chrono::duration<Rep, Period> timeout)
{
    auto start = std::chrono::system_clock::now();
    behaviourTrigger->trigger();
    while (std::chrono::system_clock::now() - start < timeout) {
        if (behaviourTrigger->behaviourFinishedRun()) {
            return true;
        }
    }
    return false;
}
} // namespace alica
