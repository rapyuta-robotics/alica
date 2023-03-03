#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <libalica-tests/util/AlicaTestsPlan.h>

namespace alica
{
class SchedulingTestPlan2 : public AlicaTestsPlan<SchedulingTestPlan2>
{
public:
    SchedulingTestPlan2(PlanContext& context);

protected:
    virtual void onInit();
    virtual void onTerminate();
};

BOOST_DLL_ALIAS(alica::SchedulingTestPlan2::create, SchedulingTestPlan2)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, SchedulingTestPlan2UtilityFunction)

} /* namespace alica */
