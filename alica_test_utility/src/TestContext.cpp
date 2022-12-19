#include "alica/test/TestContext.h"

#include "alica/test/TestBehaviourCreator.h"
#include "alica/test/Util.h"

#include <engine/BasicBehaviour.h>
#include <engine/IRoleAssignment.h>
#include <engine/model/Behaviour.h>
#include <engine/model/ConfAbstractPlanWrapper.h>
#include <engine/model/Configuration.h>
#include <engine/model/Transition.h>
#include <engine/model/TransitionCondition.h>

namespace alica::test
{
TestContext::TestContext(const std::string& agentName, const std::string& configPath, const std::string& roleSetName, const std::string& masterPlanName,
        bool stepEngine, const AgentId agentID)
        : AlicaContext(AlicaContextParams(agentName, configPath, roleSetName, masterPlanName, stepEngine, agentID))
        , _initCalled(false)
{
}

int TestContext::init(AlicaCreators& creatorCtx)
{
    AlicaCreators creators(std::move(creatorCtx.conditionCreator), std::move(creatorCtx.utilityCreator), std::move(creatorCtx.constraintCreator),
            std::move(creatorCtx.behaviourCreator), std::move(creatorCtx.planCreator), std::move(creatorCtx.transitionConditionCreator));
    return init(std::move(creators));
}

int TestContext::init(AlicaCreators&& creatorCtx)
{
    return AlicaContext::init(std::move(creatorCtx), true);
}

void TestContext::startEngine()
{
    _engine->start();
}

bool TestContext::makeBehaviourEventDriven(int64_t behaviourID)
{
    assert(_initCalled == false &&
            "[TestContext] The method makeBehaviourEventDriven(behaviourID) must be called, before the TestContext is initialised via its init-Method!");
    const Behaviour* constBehaviour = _engine->getPlanRepository().getBehaviours().find(behaviourID);
    if (constBehaviour == nullptr) {
        return false;
    }
    const_cast<Behaviour*>(constBehaviour)->setEventDriven(true);
    return true;
}

int TestContext::getTeamSize()
{
    return Util::getTeamSize(_engine.get());
}

bool TestContext::isStateActive(int64_t id) const
{
    return Util::isStateActive(_engine.get(), id);
}

bool TestContext::isPlanActive(int64_t id) const
{
    return Util::isPlanActive(_engine.get(), id);
}

BasicBehaviour* TestContext::getActiveBehaviour(const std::string& name)
{
    auto rp = getRunningPlan(name);
    if (rp != nullptr) {
        return rp->isBehaviour() ? rp->getBasicBehaviour() : nullptr;
    }
    return nullptr;
}

BasicPlan* TestContext::getActivePlan(const std::string& name)
{
    auto rp = getRunningPlan(name);
    if (rp != nullptr) {
        return rp->isBehaviour() ? nullptr : rp->getBasicPlan();
    }
    return nullptr;
}

bool TestContext::setTransitionCond(const std::string& runningPlanName, const std::string& inState, const std::string& outState)
{
    return setTransitionCond(runningPlanName, inState, outState, true);
}

bool TestContext::resetTransitionCond(const std::string& runningPlanName, const std::string& inState, const std::string& outState)
{
    return setTransitionCond(runningPlanName, inState, outState, false);
}

bool TestContext::resetAllTransitions(const std::string& runningPlanName)
{
    const auto* rp = getRunningPlan(runningPlanName);
    if (!rp) {
        return false;
    }
    const auto* plan = rp->getActivePlanAsPlan();
    if (!plan) {
        return false;
    }
    for (const auto* transition : plan->getTransitions()) {
        const auto& inState = transition->getInState()->getName();
        const auto& outState = transition->getOutState()->getName();
        resetTransitionCond(runningPlanName, inState, outState);
    }
    return true;
}

bool TestContext::isSuccess(const BasicBehaviour* beh) const
{
    return beh->isSuccess();
}

bool TestContext::isSuccess(const BasicPlan* plan) const
{
    return plan->getPlanContext()->getActiveState()->isSuccessState();
}

bool TestContext::isStateActive(const std::string& runningPlanName, const std::string& stateName)
{
    auto* rp = getRunningPlan(runningPlanName);
    if (!rp) {
        return false;
    }
    return rp->getActiveState() ? (rp->getActiveState()->getName() == stateName) : false;
}

RunningPlan* TestContext::getRunningPlan(const std::string& name)
{
    if (name.empty()) {
        return nullptr;
    }
    return name[0] == '/' ? /* fully qualified name */ followRunningPlanPath(name) : /* just name of beh/plan */ searchRunningPlanTree(name);
}

RunningPlan* TestContext::searchRunningPlanTree(const std::string& name)
{
    if (!_engine->getPlanBase().getRootNode()) {
        return nullptr;
    }
    std::vector<RunningPlan*> results;
    std::queue<RunningPlan*> q;
    q.push(_engine->getPlanBase().getRootNode());
    while (!q.empty()) {
        auto cur = q.front();
        q.pop();
        if (!cur->isActive()) {
            return nullptr;
        }
        if (getRunningPlanName(cur) == name) {
            results.push_back(cur);
        }
        for (auto child : cur->getChildren()) {
            q.push(child);
        }
    }
    if (results.empty() || results.size() > 1) {
        return nullptr;
    }
    return results.front();
}

RunningPlan* TestContext::followRunningPlanPath(const std::string& fullyQualifiedName)
{
    auto statePlanPairs = parseFullyQualifiedName(fullyQualifiedName);
    if (statePlanPairs.empty()) {
        return nullptr;
    }
    auto cur = _engine->getPlanBase().getRootNode();
    std::size_t idx = 0;
    while (cur && idx < statePlanPairs.size()) {
        const auto& [state, plan] = statePlanPairs[idx];
        if (!cur->isActive()) {
            return nullptr;
        }
        if (getActiveStateName(cur) != state) {
            return nullptr;
        }
        RunningPlan* next = nullptr;
        if (plan.empty()) {
            if (cur->getChildren().size() != 1) {
                return nullptr;
            }
            next = cur->getChildren().front();
        } else {
            for (auto childIt = cur->getChildren().begin(); childIt != cur->getChildren().end(); ++childIt) {
                if (getRunningPlanName(*childIt) == plan) {
                    next = *childIt;
                    break;
                }
            }
        }
        if (!next) {
            return nullptr;
        }
        cur = next;
    }
    return cur;
}

std::vector<std::pair<std::string, std::string>> TestContext::parseFullyQualifiedName(const std::string& fullyQualifiedName)
{
    /*
     * fullyQualifiedName (FQN) syntax
     *
     * "/<S(1),P(1)>/<S(2),P(2)>/<S(3),P(3)>"
     *
     * S(1): active state in master plan
     * S(n): active state in P(n-1)
     * P(n): active plan in S(n)
     *
     * If a state S(n) has just a single plan/behaviour attached to it, the plan/behaviour name can be ommitted along with <> symbols i.e.
     * "/S(1)/S(2)/<S(3),P(3)>""
     *
     * White spaces are stripped from the FQN
     * FQN is case sensitive
     */
    std::string fqn;
    std::copy_if(fullyQualifiedName.begin(), fullyQualifiedName.end(), std::back_inserter(fqn),
            [](const auto& ch) { return !std::isspace(static_cast<unsigned char>(ch)); });
    std::vector<std::pair<std::string, std::string>> statePlanPairs;
    std::size_t start = 1;
    while (start < fqn.length()) {
        auto end = fqn.find("/", start);
        auto count = (end == std::string::npos ? std::string::npos : end - start);
        auto splitName = fqn.substr(start, count);
        if (splitName.empty()) {
            return {};
        }
        if (splitName[0] == '<') {
            if (splitName.back() != '>') {
                return {};
            }
            auto commaPos = splitName.find(",");
            if (commaPos == std::string::npos) {
                return {};
            }
            // substring [1, commaPos)
            auto state = splitName.substr(1, commaPos - 1);
            // substring [commaPos + 1, last_char_of_splitName)
            auto plan = splitName.substr(commaPos + 1, (splitName.length() - 1) - (commaPos + 1));
            if (state.empty() || plan.empty()) {
                return {};
            }
            statePlanPairs.emplace_back(std::piecewise_construct, std::forward_as_tuple(std::move(state)), std::forward_as_tuple(std::move(plan)));
        } else {
            statePlanPairs.emplace_back(std::piecewise_construct, std::forward_as_tuple(splitName), std::forward_as_tuple());
        }
    }
    return statePlanPairs;
}

std::string TestContext::getRunningPlanName(const RunningPlan* rp)
{
    return rp->isBehaviour() ? rp->getBasicBehaviour()->getName() : rp->getActivePlan()->getName();
}

std::string TestContext::getActiveStateName(const RunningPlan* rp)
{
    return rp->getActiveState() ? rp->getActiveState()->getName() : std::string{};
}

bool TestContext::setTransitionCond(const std::string& runningPlanName, const std::string& inState, const std::string& outState, bool result)
{
    auto plan = getActivePlan(runningPlanName);
    if (!plan) {
        return false;
    }
    LockedBlackboardRW bb(*(plan->getBlackboard()));
    auto key = inState + "2" + outState;
    if (!bb.hasValue(key)) {
        return false;
    }
    bb.set(key, result);
    return true;
}

} // namespace alica::test
