#pragma once

#include "engine/AlicaClock.h"

#include <memory>

namespace alica
{

class RunningPlan;
class IAlicaWorldModel;

class BasicCondition
{
public:
    BasicCondition();
    virtual ~BasicCondition();
    virtual bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel& wm) = 0;

    bool isStateTimedOut(const AlicaTime timeOut, std::shared_ptr<RunningPlan> rp) const;
    bool isTimeOut(const AlicaTime timeOut, const AlicaTime startTime, std::shared_ptr<RunningPlan> rp) const;
};

} /* namespace alica */
