#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class MasterPlanTestConditionPlanType : public alica::BasicPlan
{
public:
    MasterPlanTestConditionPlanType(alica::PlanContext& context);
    static std::unique_ptr<MasterPlanTestConditionPlanType> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::MasterPlanTestConditionPlanType::create, MasterPlanTestConditionPlanType)

class MasterPlanTestConditionPlanTypeUtilityFunction : public alica::BasicUtilityFunction
{
public:
    MasterPlanTestConditionPlanTypeUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<MasterPlanTestConditionPlanTypeUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::MasterPlanTestConditionPlanTypeUtilityFunction::create, MasterPlanTestConditionPlanTypeUtilityFunction)

} // namespace alica::tests
