#include "PlanA.h"

#include <alica_tests/test_sched_world_model.h>
#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

PlanA::PlanA(alica::PlanContext& context)
        : BasicPlan(context)
{
    _inRunContext = false;
}

void PlanA::onInit()
{
    _wm = LockedBlackboardRW(*getGlobalBlackboard()).get<std::shared_ptr<alica_test::SchedWM>>("worldmodel");
    _wm->execOrder += "PlanA::Init\n";
    _inRunContext = true;
}

void PlanA::run()
{
    _wm->planARunCalled = true;
    if (!_inRunContext) {
        _wm->planARunOutOfOrder = true;
    }
}

void PlanA::onTerminate()
{
    _inRunContext = false;
    _wm->execOrder += "PlanA::Term\n";
}

std::unique_ptr<PlanA> PlanA::create(alica::PlanContext& context)
{
    return std::make_unique<PlanA>(context);
}

std::shared_ptr<alica::UtilityFunction> PlanAUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<PlanAUtilityFunction> PlanAUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<PlanAUtilityFunction>();
}

} // namespace alica::tests
