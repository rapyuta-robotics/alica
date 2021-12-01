#pragma once

#include "engine/BasicCondition.h"

namespace alica
{

class BasicFalseCondition : public BasicCondition
{
public:
    BasicFalseCondition();
    virtual ~BasicFalseCondition();

    bool evaluate(std::shared_ptr<RunningPlan> rp, const IAlicaWorldModel* wm);
};

} /* namespace alica */
