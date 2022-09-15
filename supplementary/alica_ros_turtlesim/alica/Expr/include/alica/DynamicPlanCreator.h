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
    DynamicPlanCreator(const std::string& defaultLibraryPath);
    virtual ~DynamicPlanCreator();
    virtual std::unique_ptr<BasicPlan> createPlan(int64_t planId, PlanContext& context) override;

private:
    std::string _defaultLibraryPath;
};

} /* namespace alica */
