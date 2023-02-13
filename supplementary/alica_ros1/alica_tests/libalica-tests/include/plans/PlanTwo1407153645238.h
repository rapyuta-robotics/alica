#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class PlanTwo : public alica::BasicPlan
{
public:
    PlanTwo(alica::PlanContext& context);
    static std::unique_ptr<PlanTwo> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::PlanTwo::create, PlanTwo)

class PlanTwoUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PlanTwoUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PlanTwoUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PlanTwoUtilityFunction::create, PlanTwoUtilityFunction)

} // namespace alica::tests
