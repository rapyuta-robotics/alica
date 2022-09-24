#pragma once
#include <engine/IPlanCreator.h>

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
};

} /* namespace alica */
