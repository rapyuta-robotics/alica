#pragma once

#include <engine/IPlanCreator.h>

#include <iostream>
#include <memory>

namespace alica
{

class BasicPlan;

class PlanCreator : public IPlanCreator
{
public:
    PlanCreator();
    virtual ~PlanCreator();
    std::unique_ptr<BasicPlan> createPlan(int64_t planId, PlanContext& context) override;
};

} /* namespace alica */
