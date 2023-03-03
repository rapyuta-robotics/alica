#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>
#include <libalica-tests/util/AlicaTestsUtilityFunction.h>

namespace alica
{
class OtherPlan : public DomainPlan
{
public:
    OtherPlan(PlanContext& context);
};

class OtherPlanUtilityFunction : public AlicaTestsUtilityFunction<OtherPlanUtilityFunction>
{
public:
    OtherPlanUtilityFunction(UtilityFunctionContext& context)
            : AlicaTestsUtilityFunction(context){};
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, OtherPlan)
BOOST_DLL_ALIAS(alica::OtherPlanUtilityFunction::create, OtherPlanUtilityFunction)
} /* namespace alica */
