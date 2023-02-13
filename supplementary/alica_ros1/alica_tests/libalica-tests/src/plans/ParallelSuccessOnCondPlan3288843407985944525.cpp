#include "ParallelSuccessOnCondPlan.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

ParallelSuccessOnCondPlan::ParallelSuccessOnCondPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void ParallelSuccessOnCondPlan::onInit() {}

std::unique_ptr<ParallelSuccessOnCondPlan> ParallelSuccessOnCondPlan::create(alica::PlanContext& context)
{
    return std::make_unique<ParallelSuccessOnCondPlan>(context);
}

std::shared_ptr<alica::UtilityFunction> ParallelSuccessOnCondPlanUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<ParallelSuccessOnCondPlanUtilityFunction> ParallelSuccessOnCondPlanUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<ParallelSuccessOnCondPlanUtilityFunction>();
}

} // namespace alica::tests
