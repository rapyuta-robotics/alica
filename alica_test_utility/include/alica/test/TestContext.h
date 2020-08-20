#pragma once

#include <engine/AlicaContext.h>

#include "BehaviourTrigger.h"

#include <engine/AlicaEngine.h>
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
     * This allows to get hold of the AlicaEngine, which is
     * an inaccessible member variable of the AlicaContext, otherwise.
     * @return Pointer to the AlicaEngine.
     */
    alica::AlicaEngine* getEngine();

    /**
     * Allows to retrieve the name of an AlicaElement.
     * @param elementID The ID of the AlicaElement, whose name is requested.
     * @return The name of the element.
     */
    template <typename AlicaElementType>
    std::string getName(uint64_t elementID);

    /**
     * This method requests the engine to step until the given state is reached, or the
     * time is over.
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
    bool prepareStepping(int64_t behaviourID);

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
    std::unique_ptr<BehaviourTrigger> setBehaviourTrigger(int64_t behaviourID, int64_t configurationID);
    std::unordered_map<std::pair<int64_t, int64_t>, std::unique_ptr<alica::test::BehaviourTrigger>, hash_pair> _behaviourTriggers;
};

template <typename AlicaElementType>
std::string TestContext::getName(uint64_t elementID)
{
    const AlicaElement* element = nullptr;
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
    } else {
        return "<ELEMENT-UNKNOWN>";
    }
    return element->getName();
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
    std::cerr << "[TestContext] Stuck in state " << _engine->getPlanBase().getDeepestNode()->getActiveState()->getName() << std::endl;
    return false;
}

template <typename Rep, typename Period>
bool TestContext::stepBehaviour(std::chrono::duration<Rep, Period> timeout, int64_t behaviourID, int64_t configurationID)
{
    auto behaviourConfKey = std::make_pair(behaviourID, configurationID);
    auto behaviourTriggerEntry = _behaviourTriggers.find(behaviourConfKey);
    if (behaviourTriggerEntry == _behaviourTriggers.end()) {
        _behaviourTriggers[behaviourConfKey] = setBehaviourTrigger(behaviourID, configurationID);
    }
    if (_behaviourTriggers[behaviourConfKey]) {
        _behaviourTriggers[behaviourConfKey]->trigger();
    } else {
        std::cerr << "[TestContext] Given <behaviourID,configurationID>-Pair is unknown in case of <" << behaviourID << ", " << configurationID << ">"
                  << std::endl;
        return false;
    }
    auto start = std::chrono::system_clock::now();
    while (std::chrono::system_clock::now() - start < timeout) {
        if (_behaviourTriggers[behaviourConfKey]->behaviourFinishedRun()) {
            return true;
        }
    }
    return false;
}

} // namespace alica::test
