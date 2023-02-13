#include "BehaviorSuccessSpamMaster.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

BehaviorSuccessSpamMaster::BehaviorSuccessSpamMaster(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void BehaviorSuccessSpamMaster::onInit() {}

std::unique_ptr<BehaviorSuccessSpamMaster> BehaviorSuccessSpamMaster::create(alica::PlanContext& context)
{
    return std::make_unique<BehaviorSuccessSpamMaster>(context);
}

std::shared_ptr<alica::UtilityFunction> BehaviorSuccessSpamMasterUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<BehaviorSuccessSpamMasterUtilityFunction> BehaviorSuccessSpamMasterUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<BehaviorSuccessSpamMasterUtilityFunction>();
}

} // namespace alica::tests
