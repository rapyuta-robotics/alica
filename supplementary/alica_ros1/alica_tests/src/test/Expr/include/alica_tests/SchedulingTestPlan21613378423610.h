#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestPlan21613378423610 : public DomainPlan
{
public:
    SchedulingTestPlan21613378423610(PlanContext& context);
    virtual ~SchedulingTestPlan21613378423610();

protected:
    virtual void onInit();
    virtual void onTerminate();
};

class UtilityFunction1613378423610 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SchedulingTestPlan21613378423610)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestPlan21613378423610UtilityFunction)

} /* namespace alica */
