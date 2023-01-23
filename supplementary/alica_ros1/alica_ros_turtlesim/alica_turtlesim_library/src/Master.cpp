#include "Master.h"
#include <engine/DefaultUtilityFunction.h>

namespace alica
{

Master::Master(PlanContext& context)
        : BasicPlan(context)
{
}
Master::~Master() {}

void Master::onInit() {}

std::shared_ptr<UtilityFunction> MasterUtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

} // namespace alica
