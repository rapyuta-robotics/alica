#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{

class HandleFailExplicit : public alica::BasicPlan
{
public:
    HandleFailExplicit(alica::PlanContext& context);
    static std::unique_ptr<HandleFailExplicit> create(alica::PlanContext& context);
    virtual void onInit() override;
};
BOOST_DLL_ALIAS(alica::tests::HandleFailExplicit::create, HandleFailExplicit)

class HandleFailExplicitUtilityFunction : public alica::BasicUtilityFunction
{
public:
    HandleFailExplicitUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<HandleFailExplicitUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::HandleFailExplicitUtilityFunction::create, HandleFailExplicitUtilityFunction)

} // namespace alica::tests
