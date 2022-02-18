#pragma once

#include <atomic>
#include <string>

#include <engine/IAlicaWorldModel.h>

namespace alica_test
{

class SchedWM : public alica::IAlicaWorldModel
{
public:
    SchedWM() { reset(); }
    SchedWM(const SchedWM&) = delete;
    ~SchedWM() = default;

    void reset()
    {
        execOrderTest = planA2PlanB = planB2PlanA = false;
        execOrder.clear();

        planARunCalled = planARunOutOfOrder = false;
        behAAARunCalled = behAAARunOutOfOrder = false;

        behAAASetSuccess = behAAASetSuccessFailed = behAAASetFailure = behAAASetFailureFailed = false;
        behAAABlockRun = false;
        behAAASuccessInInit = behAAAFailureInInit = behAAASuccessInTerminate = behAAAFailureInTerminate = false;

        executeBehaviourRunCalled = execBehaviourTest = transitionToExecuteBehaviourInSubPlan = transitionToEndTest = transitionToExecuteBehaviour = false;
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

private:
};

} // namespace alica_test
