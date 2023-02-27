#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestSequenceSubPlan11614964379654 : public DomainPlan
{
public:
    SchedulingTestSequenceSubPlan11614964379654(PlanContext& context);
    virtual ~SchedulingTestSequenceSubPlan11614964379654();
};

class UtilityFunction1614964379654 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SchedulingTestSequenceSubPlan11614964379654)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestSequenceSubPlan11614964379654UtilityFunction)

} /* namespace alica */
