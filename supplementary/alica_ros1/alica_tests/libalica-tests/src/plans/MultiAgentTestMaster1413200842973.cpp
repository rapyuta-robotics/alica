#include "MultiAgentTestMaster.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

MultiAgentTestMaster::MultiAgentTestMaster(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void MultiAgentTestMaster::onInit() {}

std::unique_ptr<MultiAgentTestMaster> MultiAgentTestMaster::create(alica::PlanContext& context)
{
    return std::make_unique<MultiAgentTestMaster>(context);
}

std::shared_ptr<alica::UtilityFunction> MultiAgentTestMasterUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MultiAgentTestMasterUtilityFunction> MultiAgentTestMasterUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MultiAgentTestMasterUtilityFunction>();
}

} // namespace alica::tests
