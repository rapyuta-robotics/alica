#pragma once

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class TestInheritBlackboardMaster : public BasicPlan
{
public:
    TestInheritBlackboardMaster(PlanContext& context);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, TestInheritBlackboardMaster)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, TestInheritBlackboardMasterUtilityFunction)

} /* namespace alica */
