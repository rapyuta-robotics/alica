#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class FrequencyTestPlan1626848999740 : public DomainPlan
{
public:
    FrequencyTestPlan1626848999740(PlanContext& context);
    virtual ~FrequencyTestPlan1626848999740();
};

class UtilityFunction1626848999740 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, FrequencyTestPlan1626848999740)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FrequencyTestPlan1626848999740UtilityFunction)
} /* namespace alica */
