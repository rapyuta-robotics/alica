#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{
class SchedWM;

class PlanBA : public alica::BasicPlan
{
public:
    PlanBA(alica::PlanContext& context);
    static std::unique_ptr<PlanBA> create(alica::PlanContext& context);
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::shared_ptr<alica_test::SchedWM> _wm;
};
BOOST_DLL_ALIAS(alica::tests::PlanBA::create, PlanBA)

class PlanBAUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PlanBAUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PlanBAUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PlanBAUtilityFunction::create, PlanBAUtilityFunction)

} // namespace alica::tests
