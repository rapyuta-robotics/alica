#pragma once

#include "engine/AlicaClock.h"
#include <memory>

using namespace std;

namespace alica {

class RunningPlan;

class BasicCondition {
public:
    BasicCondition();
    virtual ~BasicCondition();
    virtual bool evaluate(shared_ptr<RunningPlan> rp) = 0;

    bool isStateTimedOut(const AlicaTime timeOut, shared_ptr<RunningPlan> rp);
    bool isTimeOut(const AlicaTime timeOut, const AlicaTime startTime, shared_ptr<RunningPlan> rp);
};

} /* namespace alica */
