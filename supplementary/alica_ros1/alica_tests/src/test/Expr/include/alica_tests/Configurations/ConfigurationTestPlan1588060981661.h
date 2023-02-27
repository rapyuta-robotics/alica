#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ConfigurationTestPlan1588060981661 : public DomainPlan
{
public:
    ConfigurationTestPlan1588060981661(PlanContext& context);
    virtual ~ConfigurationTestPlan1588060981661();
};

class UtilityFunction1588060981661 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1588253347213 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ConfigurationTestPlan1588060981661)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ConfigurationTestPlan1588060981661UtilityFunction)
} /* namespace alica */
