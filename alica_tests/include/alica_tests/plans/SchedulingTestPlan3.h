#pragma once

#include <alica_tests/util/AlicaTestsPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class SchedulingTestPlan3 : public AlicaTestsPlan<SchedulingTestPlan3>
{
public:
    SchedulingTestPlan3(PlanContext& context);

protected:
    virtual void onInit();
    virtual void onTerminate();
};

BOOST_DLL_ALIAS(alica::SchedulingTestPlan3::create, SchedulingTestPlan3)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestPlan3UtilityFunction)

} /* namespace alica */
