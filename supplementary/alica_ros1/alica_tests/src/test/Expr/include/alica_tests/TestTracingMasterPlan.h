#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestTracingMasterPlan : public BasicPlan
{
public:
    TestTracingMasterPlan(PlanContext& context);
};
BOOST_DLL_ALIAS(alica::BasicPlan::create, TestTracingMasterPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestTracingMasterPlanUtilityFunction)

} /* namespace alica */
