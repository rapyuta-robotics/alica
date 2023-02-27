#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanFour1407153683051 : public DomainPlan
{
public:
    PlanFour1407153683051(PlanContext& context);
    virtual ~PlanFour1407153683051();
};

class UtilityFunction1407153683051 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanFour1407153683051)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanFour1407153683051UtilityFunction)
} /* namespace alica */
