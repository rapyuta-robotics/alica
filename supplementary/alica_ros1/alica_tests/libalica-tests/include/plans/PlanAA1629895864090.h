#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{
class SchedWM;

class PlanAA : public alica::BasicPlan
{
public:
    PlanAA(alica::PlanContext& context);
    static std::unique_ptr<PlanAA> create(alica::PlanContext& context);
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::shared_ptr<alica_test::SchedWM> _wm;
};
BOOST_DLL_ALIAS(alica::tests::PlanAA::create, PlanAA)

class PlanAAUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PlanAAUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PlanAAUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PlanAAUtilityFunction::create, PlanAAUtilityFunction)

} // namespace alica::tests
