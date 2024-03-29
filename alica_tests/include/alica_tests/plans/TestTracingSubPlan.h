#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestTracingSubPlan : public BasicPlan
{
public:
    TestTracingSubPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestTracingSubPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestTracingSubPlanUtilityFunction)

} /* namespace alica */
