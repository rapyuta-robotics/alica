#pragma once

#include <engine/BasicPlan.h>
#include <string>

namespace alica
{
class DomainPlan : public BasicPlan
{
public:
    DomainPlan(PlanContext& context);
    virtual ~DomainPlan();
};
} /* namespace alica */
