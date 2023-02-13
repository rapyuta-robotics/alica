#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class PlanThree : public alica::BasicPlan
{
public:
    PlanThree(alica::PlanContext& context);
    static std::unique_ptr<PlanThree> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::PlanThree::create, PlanThree)

class PlanThreeUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PlanThreeUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PlanThreeUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PlanThreeUtilityFunction::create, PlanThreeUtilityFunction)

} // namespace alica::tests
