#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class PlanOne : public alica::BasicPlan
{
public:
    PlanOne(alica::PlanContext& context);
    static std::unique_ptr<PlanOne> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::PlanOne::create, PlanOne)

class PlanOneUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PlanOneUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PlanOneUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PlanOneUtilityFunction::create, PlanOneUtilityFunction)

} // namespace alica::tests
