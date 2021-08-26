#pragma once

#include <string>

namespace alica_test
{

class SchedWM
{
public:
    static SchedWM& instance()
    {
        static SchedWM schedwm;
        return schedwm;
    }

    void reset()
    {
        execOrderTest = planA2PlanB = planB2PlanA = false;
        planARunCalled = planARunOutOfOrder = false;
        behAAARunCalled = behAAARunOutOfOrder = false;
        execOrder.clear();
    }

    bool execOrderTest;
    bool planA2PlanB;
    bool planB2PlanA;
    std::string execOrder;

    bool planARunCalled;
    bool planARunOutOfOrder;
    bool behAAARunCalled;
    bool behAAARunOutOfOrder;

private:
    SchedWM() { reset(); }
    SchedWM(const SchedWM&) = delete;
    ~SchedWM() = default;
};

}