#include "HandleFailExplicitMaster.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

HandleFailExplicitMaster::HandleFailExplicitMaster(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void HandleFailExplicitMaster::onInit() {}

std::unique_ptr<HandleFailExplicitMaster> HandleFailExplicitMaster::create(alica::PlanContext& context)
{
    return std::make_unique<HandleFailExplicitMaster>(context);
}

std::shared_ptr<alica::UtilityFunction> HandleFailExplicitMasterUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<HandleFailExplicitMasterUtilityFunction> HandleFailExplicitMasterUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<HandleFailExplicitMasterUtilityFunction>();
}

} // namespace alica::tests
