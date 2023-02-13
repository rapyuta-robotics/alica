#include "FailureHandlingMaster.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

FailureHandlingMaster::FailureHandlingMaster(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void FailureHandlingMaster::onInit() {}

std::unique_ptr<FailureHandlingMaster> FailureHandlingMaster::create(alica::PlanContext& context)
{
    return std::make_unique<FailureHandlingMaster>(context);
}

std::shared_ptr<alica::UtilityFunction> FailureHandlingMasterUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<FailureHandlingMasterUtilityFunction> FailureHandlingMasterUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<FailureHandlingMasterUtilityFunction>();
}

} // namespace alica::tests
