#pragma once

#include <engine/BasicCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class FailsOnOne : public BasicPlan
{
public:
    FailsOnOne(PlanContext& context);
};

class FailsOnOneRuntimeCondition : public BasicCondition
{
public:
    FailsOnOneRuntimeCondition(ConditionContext& context)
            : BasicCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<FailsOnOneRuntimeCondition> create(ConditionContext& context) { return std::make_unique<FailsOnOneRuntimeCondition>(context); };
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, FailsOnOne)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, FailsOnOneUtilityFunction)
BOOST_DLL_ALIAS(alica::FailsOnOneRuntimeCondition::create, FailsOnOneRuntimeCondition)
} /* namespace alica */
