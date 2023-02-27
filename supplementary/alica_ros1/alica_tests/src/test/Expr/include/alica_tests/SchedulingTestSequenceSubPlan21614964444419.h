#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestSequenceSubPlan21614964444419 : public DomainPlan
{
public:
    SchedulingTestSequenceSubPlan21614964444419(PlanContext& context);
    virtual ~SchedulingTestSequenceSubPlan21614964444419();
};

class UtilityFunction1614964444419 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, SchedulingTestSequenceSubPlan21614964444419)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestSequenceSubPlan21614964444419UtilityFunction)

} /* namespace alica */
