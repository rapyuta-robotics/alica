#include "PlanB.h"

#include <alica_tests/test_sched_world_model.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PlanB::PlanB(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void PlanB::onInit()
{
    _wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    _wm->execOrder += "PlanB::Init\n";
}

void PlanB::onTerminate()
{
    _wm->execOrder += "PlanB::Term\n";
}

std::unique_ptr<PlanB> PlanB::create(alica::PlanContext& context)
{
    return std::make_unique<PlanB>(context);
}

std::shared_ptr<alica::UtilityFunction> PlanBUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<PlanBUtilityFunction> PlanBUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PlanBUtilityFunction>();
}

} // namespace alica::tests
