#pragma once

#include "engine/BasicPlan.h"
#include "engine/BasicUtilityFunction.h"
#include <boost/dll/alias.hpp>

namespace turtlesim
{

class MakeFormation : public alica::BasicPlan
{
public:
    MakeFormation(alica::PlanContext& context);
    static std::unique_ptr<MakeFormation> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(turtlesim::MakeFormation::create, MakeFormation)

class MakeFormationUtilityFunction : public alica::BasicUtilityFunction
{
public:
    MakeFormationUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<MakeFormationUtilityFunction> create(alica::UtilityFunctionContext& context);
};
BOOST_DLL_ALIAS(turtlesim::MakeFormationUtilityFunction::create, MakeFormationUtilityFunction)

} /* namespace turtlesim */
