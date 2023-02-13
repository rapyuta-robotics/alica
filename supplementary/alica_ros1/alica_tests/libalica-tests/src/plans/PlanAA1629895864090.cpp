#include "PlanAA.h"

#include <alica_tests/test_sched_world_model.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PlanAA::PlanAA(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PlanAA::onInit()
{
    _wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    _wm->execOrder += "PlanAA::Init\n";
}

void PlanAA::onTerminate()
{
    _wm->execOrder += "PlanAA::Term\n";
}

std::unique_ptr<PlanAA> PlanAA::create(alica::PlanContext& context)
{
    return std::make_unique<PlanAA>(context);
}

std::shared_ptr<alica::UtilityFunction> PlanAAUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<PlanAAUtilityFunction> PlanAAUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PlanAAUtilityFunction>();
}

} // namespace alica::tests
