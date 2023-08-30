#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class InheritFromParentATestPlan : public AlicaTestsPlan<InheritFromParentATestPlan>
{
public:
    InheritFromParentATestPlan(PlanContext& context);

protected:
    virtual void onTerminate() override;
};

BOOST_DLL_ALIAS(alica::InheritFromParentATestPlan::create, InheritFromParentATestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, InheritFromParentATestPlanUtilityFunction)
} /* namespace alica */
