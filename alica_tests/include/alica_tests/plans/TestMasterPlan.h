#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestMasterPlan : public BasicPlan
{
public:
    TestMasterPlan(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestMasterPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestMasterPlanUtilityFunction)

} /* namespace alica */
