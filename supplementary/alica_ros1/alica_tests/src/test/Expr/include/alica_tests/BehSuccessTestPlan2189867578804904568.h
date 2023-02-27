#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class BehSuccessTestPlan2189867578804904568 : public DomainPlan
{
public:
    BehSuccessTestPlan2189867578804904568(PlanContext& context);
    virtual ~BehSuccessTestPlan2189867578804904568();
};

class UtilityFunction2189867578804904568 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, BehSuccessTestPlan2189867578804904568)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, BehSuccessTestPlan2189867578804904568UtilityFunction)
} /* namespace alica */
