#include "Master.h"
#include "world_model.hpp"

namespace turtlesim
{

Master::Master(alica::PlanContext& context)
        : alica::BasicPlan(context)
{
}
Master::~Master() {}
std::unique_ptr<Master> Master::create(alica::PlanContext& context)
{
    return std::make_unique<Master>(context);
}

void Master::onInit()
{
    ALICATurtleWorldModel::init();
}

void Master::onTerminate()
{
    ALICATurtleWorldModel::del();
}


std::shared_ptr<alica::UtilityFunction> MasterUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<MasterUtilityFunction> MasterUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<MasterUtilityFunction>();
}

} // namespace turtlesim
