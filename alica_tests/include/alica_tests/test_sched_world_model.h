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

    std::string execOrder;

private:
    SchedWM() = default;
    SchedWM(const SchedWM&) = delete;
    ~SchedWM() = default;
};

}