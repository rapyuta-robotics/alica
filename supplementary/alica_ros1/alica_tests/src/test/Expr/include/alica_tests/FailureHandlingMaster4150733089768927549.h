#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class FailureHandlingMaster4150733089768927549 : public DomainPlan
{
public:
    FailureHandlingMaster4150733089768927549(PlanContext& context);
    virtual ~FailureHandlingMaster4150733089768927549();
};

class UtilityFunction4150733089768927549 : public BasicUtilityFunction
{
    std::shared_ptr<UtilityFunction> getUtilityFunction(Plan* plan);
};
class PreCondition488794245455049811 : public DomainCondition
{
public:
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, FailureHandlingMaster4150733089768927549)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FailureHandlingMaster4150733089768927549UtilityFunction)
} /* namespace alica */
