#pragma once
#include <engine/IPlanCreator.h>

#include <functional>
#include <memory>

namespace alica
{

class BasicBehaviour;

class DynamicPlanCreator : public IPlanCreator
{
public:
    DynamicPlanCreator();
    virtual ~DynamicPlanCreator(){};
    std::unique_ptr<BasicPlan> createPlan(int64_t planId, PlanContext& context) override;

private:
    typedef std::unique_ptr<BasicPlan>(PlanCreatorType)(PlanContext&);
    std::function<PlanCreatorType> _planCreator;
    std::string _libraryPath;
};

} /* namespace alica */
