#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestPlan31613378433623 : public DomainPlan
{
public:
    SchedulingTestPlan31613378433623(PlanContext& context);
    virtual ~SchedulingTestPlan31613378433623();

protected:
    virtual void onInit();
    virtual void onTerminate();
};

class UtilityFunction1613378433623 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SchedulingTestPlan31613378433623)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestPlan31613378433623UtilityFunction)

} /* namespace alica */
