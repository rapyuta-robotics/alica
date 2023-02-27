#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanSuccessTestPlan3870436056558842479 : public DomainPlan
{
public:
    PlanSuccessTestPlan3870436056558842479(PlanContext& context);
    virtual ~PlanSuccessTestPlan3870436056558842479();
};

class UtilityFunction3870436056558842479 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanSuccessTestPlan3870436056558842479)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanSuccessTestPlan3870436056558842479UtilityFunction)
} /* namespace alica */
