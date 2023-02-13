#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{
class SchedWM;

class PlanA : public alica::BasicPlan
{
public:
    PlanA(alica::PlanContext& context);
    static std::unique_ptr<PlanA> create(alica::PlanContext& context);
    virtual void run() override;
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::atomic<bool> _inRunContext;
    std::shared_ptr<alica_test::SchedWM> _wm;
};
BOOST_DLL_ALIAS(alica::tests::PlanA::create, PlanA)

class PlanAUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PlanAUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PlanAUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PlanAUtilityFunction::create, PlanAUtilityFunction)

} // namespace alica::tests
