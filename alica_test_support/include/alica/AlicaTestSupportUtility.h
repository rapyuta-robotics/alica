#pragma once

#include <string>
#include <functional>

namespace alica
{
class AlicaEngine;
class AlicaContext;
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
    static bool stepUntilStateReached(alica::AlicaContext* ac, int64_t state, uint64_t msTimeout);
};
} // namespace alica
