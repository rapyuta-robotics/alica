#include "MakeFormation.h"

#include <engine/DefaultUtilityFunction.h>

namespace turtlesim
{

MakeFormation::MakeFormation(alica::PlanContext& context)
        : BasicPlan(context)
{
}

std::unique_ptr<MakeFormation> MakeFormation::create(alica::PlanContext& context)
{
    return std::make_unique<MakeFormation>(context);
}

std::shared_ptr<alica::UtilityFunction> MakeFormationUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    return std::make_shared<alica::DefaultUtilityFunction>(plan);
}

std::shared_ptr<MakeFormationUtilityFunction> MakeFormationUtilityFunction::create(alica::UtilityFunctionContext& context)
{
    return std::make_shared<MakeFormationUtilityFunction>();
}

} // namespace turtlesim
