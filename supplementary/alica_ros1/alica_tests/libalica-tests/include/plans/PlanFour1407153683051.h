#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class PlanFour : public alica::BasicPlan
{
public:
    PlanFour(alica::PlanContext& context);
    static std::unique_ptr<PlanFour> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::PlanFour::create, PlanFour)

class PlanFourUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PlanFourUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PlanFourUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PlanFourUtilityFunction::create, PlanFourUtilityFunction)

} // namespace alica::tests
