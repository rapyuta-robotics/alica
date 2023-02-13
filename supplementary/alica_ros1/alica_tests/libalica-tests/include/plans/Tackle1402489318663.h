#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class Tackle : public alica::BasicPlan
{
public:
    Tackle(alica::PlanContext& context);
    static std::unique_ptr<Tackle> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::Tackle::create, Tackle)

class TackleUtilityFunction : public alica::BasicUtilityFunction
{
public:
    TackleUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<TackleUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::TackleUtilityFunction::create, TackleUtilityFunction)

} // namespace alica::tests
