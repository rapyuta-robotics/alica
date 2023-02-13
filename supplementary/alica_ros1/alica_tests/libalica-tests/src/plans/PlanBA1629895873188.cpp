#include "PlanBA.h"

#include <alica_tests/test_sched_world_model.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PlanBA::PlanBA(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PlanBA::onInit()
{
    _wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    _wm->execOrder += "PlanBA::Init\n";
}

void PlanBA::onTerminate()
{
    _wm->execOrder += "PlanBA::Term\n";
}
std::unique_ptr<PlanBA> PlanBA::create(alica::PlanContext& context)
{
    return std::make_unique<PlanBA>(context);
}

std::shared_ptr<alica::UtilityFunction> PlanBAUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<PlanBAUtilityFunction> PlanBAUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PlanBAUtilityFunction>();
}

} // namespace alica::tests
