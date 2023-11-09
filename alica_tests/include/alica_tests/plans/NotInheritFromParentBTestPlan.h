#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class NotInheritFromParentBTestPlan : public AlicaTestsPlan<NotInheritFromParentBTestPlan>
{
public:
    NotInheritFromParentBTestPlan(PlanContext& context);

protected:
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::NotInheritFromParentBTestPlan::create, NotInheritFromParentBTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, NotInheritFromParentBTestPlanUtilityFunction)
} /* namespace alica */
