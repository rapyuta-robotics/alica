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

} // namespace turtlesim
