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
    DynamicPlanCreator(const std::string& defaultLibraryPath);
    virtual ~DynamicPlanCreator();
    std::unique_ptr<BasicPlan> createPlan(int64_t planId, PlanContext& context) override;

private:
    const std::string _libraryRelativePath{"/../../../lib/"};
    std::string _currentLibraryPath;

    typedef std::unique_ptr<BasicPlan>(PlanCreatorType)(PlanContext&);
    std::function<PlanCreatorType> _planCreator;
};

} /* namespace alica */
