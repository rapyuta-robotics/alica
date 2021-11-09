#pragma once

#include <engine/AlicaContext.h>

#include <engine/AlicaEngine.h>
#include <engine/BasicBehaviour.h>
#include <engine/PlanRepository.h>
#include <engine/model/Behaviour.h>
#include <engine/model/EntryPoint.h>
#include <engine/model/PlanType.h>
#include <engine/model/Task.h>
#include <engine/model/Variable.h>

#include <chrono>
#include <functional>
#include <memory>
#include <string>

namespace alica::test
{
class TestBehaviourCreator;
class TestContext : public alica::AlicaContext
{
public:
    TestContext(const std::string& agentName, const std::string& configPath,
                const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine,
                const uint64_t agentID);

    /**
     * Initialize alica framework and related modules. Note that this
     * does not start the engine, as it would in case of an AlicaContext.
     * @param creatorCtx Creator functions for utility, behaviour, constraint and condition
     * @return Return code '0' stands for success, any other for corresponding error
     * @see AlicaCreators
     * @see startEngine()
     */
    int init(AlicaCreators& creatorCtx);

    /**
     * Starts the main thread of the engine to run.
     */
    void startEngine();

    /**
     * This method requests the engine to step until the given state is reached, or the
     * time is over. This method does not consider states that the engine just passes through,
     * therefore you can only reliably check for states whose outgoing transitions are blocked.
     * @param state The state that the AlicaContext should reach before the timeout.
     * @param msTimeout The timeout in milliseconds
     * @return True, if the given state was reached before the timeout. False, otherwise.
     */
    template <typename Rep, typename Period>
    bool stepUntilStateReached(int64_t state, std::chrono::duration<Rep, Period> timeout);

    /**
     * IMPORTANT: This method must be called, before the TestContext is initialised via its init-Method!
     *
     * This method changes the eventDriven property of the referenced behaviour to be true.
     * @param behaviourID The ID of the behaviour that should be changed to execute in an event driven fashion.
     * @return True, if the property could be set. False, otherwise.
     */
    bool makeBehaviourEventDriven(int64_t behaviourID);

    /**
     * Returns a shared pointer to a BasicBehaviour, in order to enable Tests
     * to check properties of their domain specific behaviours.
     * @param behaviourID ID of the Behaviour
     * @param configurationID ID of the Configuration in order to identify the right BasicBehaviour
     * @return Shared Pointer to the requested BasicBehaviour, nullptr if behaviour is not known.
     */
    std::shared_ptr<BasicBehaviour> getBasicBehaviour(int64_t behaviourID, int64_t configurationID = 0);

    /**
     * Allows to retrieve the name of an AlicaElement.
     * @param elementID The ID of the AlicaElement, whose name is requested.
     * @return The name of the element.
     */
    template <typename AlicaElementType>
    std::string getName(uint64_t elementID);
    /**
     * Returns the size of the current team.
     * @return Team Size
     */
    int getTeamSize();

    /**
     * Checks if the state identified by id is currently active.
     * This method should be used only when engine is trigger based.
     * @param id ID of the state.
     * @return True if the state identified by id is currently active, false otherwise.
     */
    bool isStateActive(int64_t id) const;

    /**
     * Similar to isStateActive(id), this returns true if the
     * agent has an active state in the plan with the given ID.
     * @param id ID of the plan.
     * @return True if plan is active, false otherwise.
     */
    bool isPlanActive(int64_t id) const;

private:
    struct hash_pair
    {
        template <class T1, class T2>
        size_t operator()(const std::pair<T1, T2>& p) const
        {
            auto hash1 = std::hash<T1>{}(p.first);
            auto hash2 = std::hash<T2>{}(p.second);
            return hash1 ^ hash2;
        }
    };

    bool _initCalled;
};

template <typename AlicaElementType>
std::string TestContext::getName(uint64_t elementID)
{
    const AlicaElement* element = nullptr;
    std::string elementName = "<ERROR-ELEMENT-UNKNOWN>";
    if (std::is_same<AlicaElementType, alica::State>::value) {
        element = _engine->getPlanRepository().getStates().find(elementID);
    } else if (std::is_same<AlicaElementType, alica::Plan>::value) {
        element = _engine->getPlanRepository().getPlans().find(elementID);
    } else if (std::is_same<AlicaElementType, alica::PlanType>::value) {
        element = _engine->getPlanRepository().getPlanTypes().find(elementID);
    } else if (std::is_same<AlicaElementType, alica::Behaviour>::value) {
        element = _engine->getPlanRepository().getBehaviours().find(elementID);
    } else if (std::is_same<AlicaElementType, alica::Task>::value) {
        element = _engine->getPlanRepository().getTasks().find(elementID);
    } else if (std::is_same<AlicaElementType, alica::EntryPoint>::value) {
        element = _engine->getPlanRepository().getEntryPoints().find(elementID);
    } else if (std::is_same<AlicaElementType, alica::Variable>::value) {
        element = _engine->getPlanRepository().getVariables().find(elementID);
    }

    if (element) {
        elementName = element->getName();
    }

    return elementName;
}

template <typename Rep, typename Period>
bool TestContext::stepUntilStateReached(int64_t state, std::chrono::duration<Rep, Period> timeout)
{
    auto start = std::chrono::system_clock::now();
    while (std::chrono::system_clock::now() - start < timeout) {
        stepEngine();
        if (isStateActive(state)) {
            return true;
        }
    }
    std::cerr << "[TestContext] Stuck in state: " << std::endl << _engine->getPlanBase().getDeepestNode()->getActiveState()->toString() << std::endl;
    return false;
}

} // namespace alica::test
