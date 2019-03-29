#pragma once

#include <string>

#include "Condition.h"

namespace alica
{
class RunningPlan;

class RuntimeCondition : public Condition
{
public:
    RuntimeCondition();
    virtual ~RuntimeCondition();

    std::string toString() const override;
};

} // namespace alica
