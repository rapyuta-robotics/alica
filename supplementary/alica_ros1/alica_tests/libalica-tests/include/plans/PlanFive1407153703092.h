#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class PlanFive : public alica::BasicPlan
{
public:
    PlanFive(alica::PlanContext& context);
    static std::unique_ptr<PlanFive> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::PlanFive::create, PlanFive)

class PlanFiveUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PlanFiveUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PlanFiveUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PlanFiveUtilityFunction::create, PlanFiveUtilityFunction)

} // namespace alica::tests
