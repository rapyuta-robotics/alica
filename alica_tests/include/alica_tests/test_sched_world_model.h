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
        execOrder.clear();
    }

    bool execOrderTest;
    bool planA2PlanB;
    bool planB2PlanA;
    std::string execOrder;

private:
    SchedWM() { reset(); }
    SchedWM(const SchedWM&) = delete;
    ~SchedWM() = default;
};

}