#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class ReadConfigurationPlan1588061334567 : public DomainPlan
{
public:
    ReadConfigurationPlan1588061334567(PlanContext& context);
    virtual ~ReadConfigurationPlan1588061334567();
};

class UtilityFunction1588061334567 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition1588069612661 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};
class PreCondition1588069615553 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, ReadConfigurationPlan1588061334567)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, ReadConfigurationPlan1588061334567UtilityFunction)
} /* namespace alica */
