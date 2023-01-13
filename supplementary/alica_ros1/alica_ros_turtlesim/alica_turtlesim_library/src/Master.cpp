#include "Master.h"
#include <engine/DefaultUtilityFunction.h>

namespace alica
{

Master::Master(PlanContext& context)
        : BasicPlan(context)
{
    std::cerr << "Masterr created" << std::endl;
}
Master::~Master() {}

std::shared_ptr<UtilityFunction> MasterUtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}
} // namespace alica
