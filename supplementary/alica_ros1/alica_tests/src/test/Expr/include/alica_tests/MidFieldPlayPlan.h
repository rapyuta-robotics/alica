#pragma once

#include <alica_tests/DomainCondition.h>

#include <boost/dll/alias.hpp>
#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/DefaultUtilityFunction.h>
#include <engine/UtilityFunction.h>

namespace alica
{
class MidFieldPlayPlan : public BasicPlan
{
public:
    MidFieldPlayPlan(PlanContext& context);
};
class MidFieldPlayPlanRuntimeCondition : public DomainCondition
{
public:
    MidFieldPlayPlanRuntimeCondition(ConditionContext& context)
            : DomainCondition(){};
    bool evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb);
    static std::unique_ptr<MidFieldPlayPlanRuntimeCondition> create(ConditionContext& context)
    {
        return std::make_unique<MidFieldPlayPlanRuntimeCondition>(context);
    };
};

BOOST_DLL_ALIAS(alica::BasicPlan::create, MidFieldPlayPlan)
BOOST_DLL_ALIAS(alica::BasicUtilityFunction::create, MidFieldPlayPlanUtilityFunction)
BOOST_DLL_ALIAS(alica::MidFieldPlayPlanRuntimeCondition::create, MidFieldPlayPlanRuntimeCondition)
} /* namespace alica */
