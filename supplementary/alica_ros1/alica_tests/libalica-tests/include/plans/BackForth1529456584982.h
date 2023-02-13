#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class BackForth : public alica::BasicPlan
{
public:
    BackForth(alica::PlanContext& context);
    static std::unique_ptr<BackForth> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::BackForth::create, BackForth)

class BackForthUtilityFunction : public alica::BasicUtilityFunction
{
public:
    BackForthUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<BackForthUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::BackForthUtilityFunction::create, BackForthUtilityFunction)

} // namespace alica::tests
