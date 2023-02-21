#include <engine/DefaultUtilityFunction.h>
#include <libalica-supplementary-tests/VariableHandling/VHMaster.h>

namespace alica
{
VHMaster::VHMaster(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> VHMasterUtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

void VHMasterRuntimeConditionConstraint::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp) {}

bool VHMasterRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

std::unique_ptr<VHMaster> VHMaster::create(PlanContext& context)
{
    return std::make_unique<VHMaster>(context);
}
std::unique_ptr<VHMasterUtilityFunction> VHMasterUtilityFunction::create(UtilityFunctionContext& context)
{
    return std::make_unique<VHMasterUtilityFunction>();
}

std::unique_ptr<VHMasterRuntimeCondition> VHMasterRuntimeCondition::create(alica::ConditionContext& context)
{
    return std::make_unique<VHMasterRuntimeCondition>();
}

std::unique_ptr<VHMasterRuntimeConditionConstraint> VHMasterRuntimeConditionConstraint::create(alica::ConstraintContext& context)
{
    return std::make_unique<VHMasterRuntimeConditionConstraint>();
}

} // namespace alica
