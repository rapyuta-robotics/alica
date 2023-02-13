#pragma once

#include <engine/BasicPlan.h>
#include <engine/BasicUtilityFunction.h>
#include <engine/RunningPlan.h>
#include <engine/blackboard/Blackboard.h>

#include <boost/dll/alias.hpp>

namespace alica::tests
{
class SchedWM;

class PlanB : public alica::BasicPlan
{
public:
    PlanB(alica::PlanContext& context);
    static std::unique_ptr<PlanB> create(alica::PlanContext& context);
    virtual void onInit() override;
    virtual void onTerminate() override;

private:
    std::shared_ptr<alica_test::SchedWM> _wm;
};
BOOST_DLL_ALIAS(alica::tests::PlanB::create, PlanB)

class PlanBUtilityFunction : public alica::BasicUtilityFunction
{
public:
    PlanBUtilityFunction() = default;
    std::shared_ptr<alica::UtilityFunction> getUtilityFunction(alica::Plan* plan) override;
    static std::shared_ptr<PlanBUtilityFunction> create(alica::UtilityFunctionContext&);
};
BOOST_DLL_ALIAS(alica::tests::PlanBUtilityFunction::create, PlanBUtilityFunction)

} // namespace alica::tests
