#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class AttackPlan : public alica::BasicPlan
{
public:
    AttackPlan(alica::PlanContext& context);
    static std::unique_ptr<AttackPlan> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::AttackPlan::create, AttackPlan)

class AttackPlanUtilityFunction : public alica::BasicUtilityFunction
{
public:
    AttackPlanUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<AttackPlanUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::AttackPlanUtilityFunction::create, AttackPlanUtilityFunction)

} // namespace alica::tests
