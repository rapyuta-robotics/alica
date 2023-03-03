#pragma once

#include <alica_tests/DomainCondition.h>
#include <alica_tests/DomainPlan.h>
#include <boost/dll/alias.hpp>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class FailsOnOne : public DomainPlan
{
public:
    FailsOnOne(PlanContext& context);
};

class FailsOnOneRuntimeCondition : public DomainCondition
{
public:
    FailsOnOneRuntimeCondition(ConditionContext& context)
            : DomainCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<FailsOnOneRuntimeCondition> create(ConditionContext& context) { return std::make_unique<FailsOnOneRuntimeCondition>(context); };
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, FailsOnOne)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FailsOnOneUtilityFunction)
BOOST_DLL_ALIAS(alica::FailsOnOneRuntimeCondition::create, FailsOnOneRuntimeCondition)
} /* namespace alica */
