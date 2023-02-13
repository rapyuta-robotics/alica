#include "ConstraintTestMaster.h"

#include <engine/DefaultUtilityFunction.h>

namespace alica::tests
{

ConstraintTestMaster::ConstraintTestMaster(alica::PlanContext& context)
        : BasicPlan(context)
{
}

void ConstraintTestMaster::onInit() {}

std::unique_ptr<ConstraintTestMaster> ConstraintTestMaster::create(alica::PlanContext& context)
{
    return std::make_unique<ConstraintTestMaster>(context);
}

std::shared_ptr<alica::UtilityFunction> ConstraintTestMasterUtilityFunction::getUtilityFunction(alica::Plan* plan)
{
    std::shared_ptr<alica::UtilityFunction> defaultFunction = std::make_shared<alica::DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::shared_ptr<ConstraintTestMasterUtilityFunction> ConstraintTestMasterUtilityFunction::create(alica::UtilityFunctionContext&)
{
    return std::make_shared<ConstraintTestMasterUtilityFunction>();
}

} // namespace alica::tests
