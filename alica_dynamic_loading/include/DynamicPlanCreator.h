#pragma once
#include <engine/IPlanCreator.h>

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

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
    std::unordered_map<std::string, std::function<PlanCreatorType>> _planCreatorMap; // See DynamicBehaviourCreator for an explanation
    std::vector<std::string> _libraryPath;
};

} /* namespace alica */
