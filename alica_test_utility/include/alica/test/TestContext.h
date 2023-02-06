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
    TestContext(const std::string& agentName, const std::string& configPath, const std::string& roleSetName, const std::string& masterPlanName, bool stepEngine,
            const AgentId agentID = InvalidAgentID);

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
     * Initialize alica framework and related modules.
     *
     * @param creatorCtx Creator functions for utility, behaviour, constraint and condition
     *
     * @return Return code '0' stands for success, any other for corresponding error
     *
     * @see AlicaCreators
     */
    int init(AlicaCreators&& creatorCtx);

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
    [[deprecated("Use STEP_UNTIL(isStateActive(running_plan_name, state_name)) instead")]] bool stepUntilStateReached(
            int64_t state, std::chrono::duration<Rep, Period> timeout);

    /**
     * IMPORTANT: This method must be called, before the TestContext is initialised via its init-Method!
     *
     * This method changes the eventDriven property of the referenced behaviour to be true.
     * @param behaviourID The ID of the behaviour that should be changed to execute in an event driven fashion.
     * @return True, if the property could be set. False, otherwise.
     */
    bool makeBehaviourEventDriven(int64_t behaviourID);

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
    [[deprecated("Use isStateActive(state_name) instead")]] bool isStateActive(int64_t id) const;

    /**
     * Similar to isStateActive(id), this returns true if the
     * agent has an active state in the plan with the given ID.
     * @param id ID of the plan.
     * @return True if plan is active, false otherwise.
     */
    [[deprecated("Use getActivePlan(plan_name) instead")]] bool isPlanActive(int64_t id) const;

    /**
     * Get the behaviour's runnable object given its name
     * @param name Either the fully qualified name or just the name of the behaviour
     * @return Pointer to the runnable object or nullptr if the behaviour is not found or not active
     * nullptr is also returned if there are multiple behaviours active with the same name. Use the
     * fully qualified name to retrieve the behaviour in this case
     * The reason for failure can be retrieved by calling getLastFailure()
     */
    BasicBehaviour* getActiveBehaviour(const std::string& name);

    /**
     * Get the plan's runnable object given its name
     * @param name Either the fully qualified name or just the name of the plan
     * @return Pointer to the runnable object or nullptr if the plan is not found or not active
     * nullptr is also returned if there are multiple plans active with the same name. Use the
     * fully qualified name to retrieve the plan in this case
     * The reason for failure can be retrieved by calling getLastFailure()
     */
    BasicPlan* getActivePlan(const std::string& name);

    /**
     * Set a transition defined by the states it connects in the given running plan
     * This method requires the blackboard key with the name `<in_state_name>2<out_state_name>` to be defined in the plan's
     * blackboard & the TriggerFromInputCond condition to be used for the transition
     * @param runningPlanName The name (fully qualified if required) of the plan in which the transition is present. This plan should be active
     * @param inState The name of the incoming state of the transition
     * @param outState The name of the outgoing state of the transition
     * @return True if the transition could be set correctly, false otherwise
     * Possible reasons for failure could be:
     * 1. inState or outState is not found
     * 2. Corresponding blackboard key is not found
     * 3. Running plan is not not a plan or is not active or does not exist
     */
    bool setTransitionCond(const std::string& runningPlanName, const std::string& inState, const std::string& outState);

    /**
     * Reset's a transition. Refer to setTransitionCond for details
     */
    bool resetTransitionCond(const std::string& runningPlanName, const std::string& inState, const std::string& outState);

    /**
     * Reset all transitions in the given plan, notes:
     * The plan should be active
     * Typically this method has to be called in all plans that use the TriggerFromInputCond condition in the init
     * method of the plan, so that the transitions evaluate to false until they are set
     */
    bool resetAllTransitions(const std::string& runningPlanName);

    /**
     * Reset all transitions in the given running plan, refer resetAllTransitions(name) for details
     */
    bool resetAllTransitions(RunningPlan* rp);

    // Check for behaviour/plan success
    bool isSuccess(const BasicBehaviour* beh) const;
    bool isSuccess(const BasicPlan* plan) const;

    // Check if a state is active in the given plan
    bool isStateActive(const std::string& runningPlanName, const std::string& stateName);

    // Retrieve the last failure that occured when any of the other API's are called
    std::string getLastFailure() const { return _lastFailureInfo; }

private:
    template <class... Args>
    void setFailureInfo(Args&&... args) const
    {
        std::ostringstream oss;
        (oss << ... << std::forward<Args>(args));
        _lastFailureInfo = oss.str();
    }

    RunningPlan* getRunningPlan(const std::string& name);
    RunningPlan* searchRunningPlanTree(const std::string& name);
    RunningPlan* followRunningPlanPath(const std::string& fullyQualifiedName);
    std::vector<std::pair<std::string /* state name */, std::string /* plan/beh name */>> parseFullyQualifiedName(const std::string& fullyQualifiedName) const;
    static std::string getRunningPlanName(const RunningPlan* rp);
    static std::string getActiveStateName(const RunningPlan* rp);
    bool setTransitionCond(const std::string& runningPlanName, const std::string& inState, const std::string& outState, bool result);
    bool setTransitionCond(RunningPlan* rp, const std::string& inState, const std::string& outState, bool result);

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
    mutable std::string _lastFailureInfo;
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
        try {
            stepEngine();
        } catch (const std::exception& e) {
            std::cerr << "[TestContext] Exception in state.  Halting steps" << std::endl;
            break;
        }
        if (isStateActive(state)) {
            return true;
        }
    }
    std::cerr << "[TestContext] Stuck in state: " << std::endl << _engine->getPlanBase().getDeepestNode()->getActiveState()->toString() << std::endl;
    return false;
}

} // namespace alica::test
