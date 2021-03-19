#pragma once

#include <string>

namespace alica
{
class BasicPlan
{
public:
    BasicPlan();
    virtual ~BasicPlan() = default;

    virtual void init(){};
    virtual void onTermination(){};
};
} // namespace alica
