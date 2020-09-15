#pragma once

#include <engine/AlicaContext.h>

#include "BehaviourTrigger.h"

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
class BehaviourTrigger;
class TestContext : public alica::AlicaContext
{
public:
    TestContext(const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine, const essentials::IdentifierConstPtr agentID = nullptr);

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
     * Adds the given trigger to the lookup table of triggers, if none is known already for
     * the given behaviour and configuration ID pair.
     * @param behaviourID ID of the Behaviour
     * @param configurationID ID of the Configuration in order to identify the right BasicBehaviour
     * @param trigger Optional trigger, if not given, a standard trigger is created and returned.
     * @return Returns the created trigger, if no trigger was passed. Otherwise, the passed trigger is returned.
     */
    std::shared_ptr<essentials::ITrigger> addBehaviourTrigger(
            int64_t behaviourID, int64_t configurationID = 0, std::shared_ptr<essentials::ITrigger> trigger = nullptr);

    /**
     * Returns a shared pointer to a BasicBehaviour, in order to enable Tests
     * to check properties of their domain specific behaviours.
     * @param behaviourID ID of the Behaviour
     * @param configurationID ID of the Configuration in order to identify the right BasicBehaviour
     * @return Shared Pointer to the requested BasicBehaviour, nullptr if behaviour is not known.
     */
    std::shared_ptr<BasicBehaviour> getBasicBehaviour(int64_t behaviourID, int64_t configurationID = 0);

    /**
     * Triggers the given behaviour to execute its run method one time.
     *
     * @param timeout Timeout for waiting the behaviour to finish its run method.
     * @param behaviourID The ID of the behaviour that should be stepped.
     * @param configurationID Optional parameter for distinguishing different BasicBehaviour instances. 0 identifies
     * all BasicBehaviour instances that have no configuration attached.
     * @return True, if the behaviour finished its run method in time. False, in case of timeout or unknown IDs.
     */
    template <typename Rep, typename Period>
    bool stepBehaviour(std::chrono::duration<Rep, Period> timeout, int64_t behaviourID, int64_t configurationID = 0);

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
     * Provides access to the local agent.
     * @return const pointer to the local agent.
     */
    const alica::Agent* getLocalAgent();

    /**
     * Provides access to the agent with the given ID.
     * @param agentID
     * @return const pointer to the agent with the given ID.
     */
    const alica::Agent* getAgentByID(essentials::IdentifierConstPtr agentID);

    /**
     * Just like isStateActive(id), this returns true if the
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
    bool isPlanActiveHelper(const RunningPlan* rp, int64_t id) const;

    std::unordered_map<std::pair<int64_t, int64_t>, std::shared_ptr<essentials::ITrigger>, hash_pair> _behaviourTriggers;
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

template <typename Rep, typename Period>
bool TestContext::stepBehaviour(std::chrono::duration<Rep, Period> timeout, int64_t behaviourID, int64_t configurationID)
{
    auto behaviourConfKey = std::make_pair(behaviourID, configurationID);
    auto behaviourTriggerEntry = _behaviourTriggers.find(behaviourConfKey);

    if (behaviourTriggerEntry != _behaviourTriggers.end()) {
        _behaviourTriggers[behaviourConfKey]->run(false);
    } else {
        std::cerr << "[TestContext] There is no trigger known for the given <behaviourID,configurationID>-Pair in case of <" << behaviourID << ", "
                  << configurationID << ">. Either trigger it yourself or register the trigger to the TestContext first." << std::endl;
    }

    auto basicBehaviour = getBasicBehaviour(behaviourID, configurationID);
    auto start = std::chrono::system_clock::now();
    while (std::chrono::system_clock::now() - start < timeout) {
        if (basicBehaviour->isTriggeredRunFinished()) {
            return true;
        }
    }
    return false;
}

} // namespace alica::test
