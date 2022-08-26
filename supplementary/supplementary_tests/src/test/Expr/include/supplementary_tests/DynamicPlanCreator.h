#pragma once
#include <engine/IPlanCreator.h>

#include <iostream>
#include <memory>

namespace alica
{

class BasicBehaviour;

class DynamicPlanCreator : public IPlanCreator
{
public:
    DynamicPlanCreator();
    virtual ~DynamicPlanCreator();
    virtual std::unique_ptr<BasicPlan> createPlan(int64_t planId, PlanContext& context) override;
};

} /* namespace alica */
