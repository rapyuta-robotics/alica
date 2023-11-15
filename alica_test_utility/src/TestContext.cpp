#include "alica/test/TestContext.h"

#include "alica/test/TestBehaviourCreator.h"
#include "alica/test/Util.h"

#include <engine/BasicBehaviour.h>
#include <engine/IRoleAssignment.h>
#include <engine/model/Behaviour.h>
#include <engine/model/ConfAbstractPlanWrapper.h>
#include <engine/model/Transition.h>
#include <engine/model/TransitionCondition.h>

namespace alica::test
{

TestContext::TestContext(const std::string& agentName, const std::string& configPath, const std::string& roleSetName, const std::string& masterPlanName,
        bool stepEngine, const AgentId agentID)
        : AlicaContext(AlicaContextParams(agentName, std::vector<std::string>{configPath}, roleSetName, masterPlanName, stepEngine, agentID))
{
}

TestContext::TestContext(const std::string& agentName, const std::vector<std::string>& configPaths, const std::string& roleSetName,
        const std::string& masterPlanName, bool stepEngine, const AgentId agentID, std::optional<std::string> placeholderMapping)
        : AlicaContext(AlicaContextParams(agentName, configPaths, roleSetName, masterPlanName, stepEngine, agentID, placeholderMapping))
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
        if (!rp->isBehaviour()) {
            setFailureInfo(name, " exists, but it is not a behaviour");
            return nullptr;
        }
        return rp->getBasicBehaviour();
    }
    return nullptr;
}

BasicPlan* TestContext::getActivePlan(const std::string& name)
{
    auto rp = getRunningPlan(name);
    if (rp != nullptr) {
        if (rp->isBehaviour()) {
            setFailureInfo(name, " is not a plan, it is a behaviour");
            return nullptr;
        }
        return rp->getBasicPlan();
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
    auto* rp = getRunningPlan(runningPlanName);
    if (!rp) {
        return false;
    }
    return resetAllTransitions(rp);
}

bool TestContext::resetAllTransitions(RunningPlan* rp)
{
    const auto* plan = rp->getActivePlanAsPlan();
    if (!plan) {
        setFailureInfo("could not reset transitions since the running plan is not a plan but a behaviour");
        return false;
    }
    for (const auto* transition : plan->getTransitions()) {
        const auto& inState = transition->getInState()->getName();
        const auto& outState = transition->getOutState()->getName();
        setTransitionCond(rp, inState, outState, false);
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
    if (!rp->getActiveState()) {
        setFailureInfo(runningPlanName, " exists, but it has no active state");
        return false;
    }
    if (rp->getActiveState()->getName() != stateName) {
        setFailureInfo(runningPlanName, "'s actual active state: ", rp->getActiveState()->getName(), ", expected active state: ", stateName);
        return false;
    }
    return true;
}

RunningPlan* TestContext::getRunningPlan(const std::string& name)
{
    if (name.empty()) {
        setFailureInfo("name cannot be empty");
        return nullptr;
    }
    return name[0] == '/' ? /* fully qualified name */ followRunningPlanPath(name) : /* just name of beh/plan */ searchRunningPlanTree(name);
}

RunningPlan* TestContext::searchRunningPlanTree(const std::string& name)
{
    if (!_engine->getPlanBase().getRootNode()) {
        setFailureInfo("running plan tree is empty");
        return nullptr;
    }
    std::vector<RunningPlan*> results;
    std::queue<RunningPlan*> q;
    q.push(_engine->getPlanBase().getRootNode());
    while (!q.empty()) {
        auto cur = q.front();
        q.pop();
        if (!cur->isActive()) {
            continue;
        }
        if (getRunningPlanName(cur) == name) {
            results.push_back(cur);
        }
        for (auto child : cur->getChildren()) {
            q.push(child);
        }
    }
    if (results.empty()) {
        setFailureInfo("running plan with name: ", name, " is not active or is not found");
        return nullptr;
    }
    if (results.size() > 1) {
        setFailureInfo("more than one running plan with name: ", name, " is active");
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
            setFailureInfo("plan ", plan, " in state ", state, " is not active");
            return nullptr;
        }
        if (getActiveStateName(cur) != state) {
            setFailureInfo("state ", state, " in plan ", plan, " is not active");
            return nullptr;
        }
        RunningPlan* next = nullptr;
        if (plan.empty()) {
            if (cur->getChildren().size() != 1) {
                setFailureInfo("more than 1 plan in state ", state, ", however plan is not specified");
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
            setFailureInfo("plan ", plan, " in state ", state, " is not found");
            return nullptr;
        }
        cur = next;
        ++idx;
    }
    return cur;
}

std::vector<std::pair<std::string, std::string>> TestContext::parseFullyQualifiedName(const std::string& fullyQualifiedName) const
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
        auto count = (end == std::string::npos ? fqn.length() - start : end - start);
        auto splitName = fqn.substr(start, count);
        if (splitName.empty()) {
            setFailureInfo("parse failure, `/` not found");
            return {};
        }
        if (splitName[0] == '<') {
            if (splitName.back() != '>') {
                setFailureInfo("parse failure, `>` not found");
                return {};
            }
            auto commaPos = splitName.find(",");
            if (commaPos == std::string::npos) {
                setFailureInfo("parse failure, `,` not found");
                return {};
            }
            // substring [1, commaPos)
            auto state = splitName.substr(1, commaPos - 1);
            // substring [commaPos + 1, last_char_of_splitName)
            auto plan = splitName.substr(commaPos + 1, (splitName.length() - 1) - (commaPos + 1));
            if (state.empty()) {
                setFailureInfo("parse failure, state not specified");
                return {};
            } else if (plan.empty()) {
                setFailureInfo("parse failure, plan not specified");
                return {};
            }
            statePlanPairs.emplace_back(std::piecewise_construct, std::forward_as_tuple(std::move(state)), std::forward_as_tuple(std::move(plan)));
        } else {
            statePlanPairs.emplace_back(std::piecewise_construct, std::forward_as_tuple(splitName), std::forward_as_tuple());
        }
        start = (end == std::string::npos) ? fqn.length() : end + 1;
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
    auto rp = getRunningPlan(runningPlanName);
    if (!rp) {
        return false;
    }
    return setTransitionCond(rp, inState, outState, result);
}

bool TestContext::setTransitionCond(RunningPlan* rp, const std::string& inState, const std::string& outState, bool result)
{
    auto plan = rp->getBasicPlan();
    if (!plan) {
        setFailureInfo("cannot set transition result, running plan is not a plan");
        return false;
    }
    LockedBlackboardRW bb(*(plan->getBlackboard()));
    auto key = inState + "2" + outState;
    if (!bb.hasValue(key)) {
        setFailureInfo("plan does not have key ", key, " to set the corresponding transition's result");
        return false;
    }
    bb.set(key, result);
    return true;
}

} // namespace alica::test
