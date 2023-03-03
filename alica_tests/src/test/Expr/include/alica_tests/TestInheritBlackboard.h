#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestInheritBlackboard : public BasicPlan
{
public:
    TestInheritBlackboard(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestInheritBlackboard)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestInheritBlackboardUtilityFunction)

} /* namespace alica */
