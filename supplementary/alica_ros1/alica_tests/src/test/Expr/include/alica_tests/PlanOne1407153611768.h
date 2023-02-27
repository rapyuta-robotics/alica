#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class PlanOne1407153611768 : public DomainPlan
{
public:
    PlanOne1407153611768(PlanContext& context);
    virtual ~PlanOne1407153611768();
};

class UtilityFunction1407153611768 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, PlanOne1407153611768)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, PlanOne1407153611768UtilityFunction)
} /* namespace alica */
