#include "MasterSyncTransition.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

MasterSyncTransition::MasterSyncTransition(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void MasterSyncTransition::onInit() {}

std::unique_ptr<MasterSyncTransition> MasterSyncTransition::create(alica::PlanContext& context)
{
    return std::make_unique<MasterSyncTransition>(context);
}

std::shared_ptr<alica::UtilityFunction> MasterSyncTransitionUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MasterSyncTransitionUtilityFunction> MasterSyncTransitionUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MasterSyncTransitionUtilityFunction>();
}

} // namespace alica::tests
