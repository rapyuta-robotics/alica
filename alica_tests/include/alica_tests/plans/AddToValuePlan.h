#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class AddToValuePlan : public AlicaTestsPlan<AddToValuePlan>
{
public:
    AddToValuePlan(PlanContext& context);

protected:
    virtual void onInit() override;
};

BOOST_DLL_ALIAS(alica::AddToValuePlan::create, AddToValuePlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, AddToValuePlanUtilityFunction)
} /* namespace alica */
