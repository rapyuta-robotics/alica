#pragma once

#include <atomic>
#include <string>
#include <unordered_map>
#include <vector>

#include <engine/IAlicaWorldModel.h>

namespace alica_test
{

class SchedWM : public alica::IAlicaWorldModel
{
public:
    SchedWM() { reset(); }
    SchedWM(const SchedWM&) = delete;
    ~SchedWM() = default;

    std::vector<double> robotsXPos;
    std::vector<std::pair<std::string, std::string>> tracingTags;
    std::vector<std::pair<std::string, std::string>> tracingLogs;
    std::unordered_map<std::string, std::string> tracingParents;

    void reset()
    {
        tracingLogs.clear();
        tracingTags.clear();
        tracingParents.clear();

        execOrderTest = planA2PlanB = planB2PlanA = false;
        execOrder.clear();

        planARunCalled = planARunOutOfOrder = false;
        behAAARunCalled = behAAARunOutOfOrder = false;

        behAAASetSuccess = behAAASetSuccessFailed = behAAASetFailure = behAAASetFailureFailed = false;
        behAAABlockRun = false;
        behAAASuccessInInit = behAAAFailureInInit = behAAASuccessInTerminate = behAAAFailureInTerminate = false;

        executeBehaviourRunCalled = execBehaviourTest = transitionToExecuteBehaviourInSubPlan = transitionToEndTest = transitionToExecuteBehaviour = false;

        preCondition1840401110297459509=false;
    }

    bool execOrderTest;
    bool planA2PlanB;
    bool planB2PlanA;
    std::string execOrder;

    std::atomic<bool> planARunCalled;
    std::atomic<bool> planARunOutOfOrder;
    std::atomic<bool> behAAARunCalled;
    std::atomic<bool> behAAARunOutOfOrder;

    std::atomic<bool> behAAASetSuccess;
    std::atomic<bool> behAAASetSuccessFailed;
    std::atomic<bool> behAAASetFailure;
    std::atomic<bool> behAAASetFailureFailed;
    std::atomic<bool> behAAABlockRun;
    std::atomic<bool> behAAASuccessInInit;
    std::atomic<bool> behAAAFailureInInit;
    std::atomic<bool> behAAASuccessInTerminate;
    std::atomic<bool> behAAAFailureInTerminate;

    std::atomic<bool> executeBehaviourRunCalled;
    std::atomic<bool> execBehaviourTest;
    std::atomic<bool> transitionToExecuteBehaviourInSubPlan;
    std::atomic<bool> transitionToExecuteBehaviour;
    std::atomic<bool> transitionToEndTest;

    std::atomic<bool> preCondition1840401110297459509;

private:
};

} // namespace alica_test
