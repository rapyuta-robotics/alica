#pragma once

#include <memory>

namespace alica
{

class BasicPlan;
struct PlanContext;

class IPlanCreator
{
public:
    virtual ~IPlanCreator() {}
    virtual std::unique_ptr<BasicPlan> createPlan(int64_t planId, PlanContext& context) = 0;
};

} /* namespace alica */
