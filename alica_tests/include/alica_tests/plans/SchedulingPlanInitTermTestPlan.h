#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica
{
class SchedulingPlanInitTermTestPlan : public AlicaTestsPlan<SchedulingPlanInitTermTestPlan>
{
public:
    SchedulingPlanInitTermTestPlan(PlanContext& context);

protected:
    virtual void onInit() override;
    virtual void onTerminate() override;
};

BOOST_DLL_ALIAS(alica::SchedulingPlanInitTermTestPlan::create, SchedulingPlanInitTermTestPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingPlanInitTermTestPlanUtilityFunction)
} /* namespace alica */
