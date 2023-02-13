#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class Defend : public alica::BasicPlan
{
public:
    Defend(alica::PlanContext& context);
    static std::unique_ptr<Defend> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::Defend::create, Defend)

class DefendUtilityFunction : public alica::BasicUtilityFunction
{
public:
    DefendUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<DefendUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::DefendUtilityFunction::create, DefendUtilityFunction)

} // namespace alica::tests
