#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class FailsOnOne : public alica::BasicPlan
{
public:
    FailsOnOne(alica::PlanContext& context);
    static std::unique_ptr<FailsOnOne> create(alica::PlanContext& context);
};
BOOST_DLL_ALIAS(alica::tests::FailsOnOne::create, FailsOnOne)

class FailsOnOneUtilityFunction : public alica::BasicUtilityFunction
{
public:
    FailsOnOneUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<FailsOnOneUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::FailsOnOneUtilityFunction::create, FailsOnOneUtilityFunction)

} // namespace alica::tests
