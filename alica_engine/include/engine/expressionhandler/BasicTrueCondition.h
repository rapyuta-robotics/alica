#pragma once

#include "engine/BasicCondition.h"

namespace alica
{

class BasicTrueCondition : public BasicCondition
{
public:
    BasicTrueCondition();
    virtual ~BasicTrueCondition();

    bool evaluate(std::shared_ptr<RunningPlan> rp);
};

} /* namespace alica */