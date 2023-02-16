#include <libalica-supplementary-tests/ProblemModule/ProblemBuildingMaster.h>
#include <assert.h>
#include <autodiff/AutoDiff.h>
#include <engine/constraintmodul/ProblemDescriptor.h>
using autodiff::TermPtr;

namespace alica
{
ProblemBuildingMaster::ProblemBuildingMaster(PlanContext& context)
        : BasicPlan(context)
{
}
ProblemBuildingMaster::~ProblemBuildingMaster()
{
}

std::shared_ptr<UtilityFunction> ProblemBuildingMasterUtilityFunction::getUtilityFunction(Plan* plan)
{
    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

void ProblemBuildingMasterPreConditionConstraint::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
}

std::unique_ptr<ProblemBuildingMaster> ProblemBuildingMaster::create(alica::PlanContext& context)
{
    return std::make_unique<ProblemBuildingMaster>(context);
}

std::unique_ptr<ProblemBuildingMasterUtilityFunction> ProblemBuildingMasterUtilityFunction::create(alica::UtilityFunctionContext& context)
{
    return std::make_unique<ProblemBuildingMasterUtilityFunction>();
}

std::unique_ptr<ProblemBuildingMasterPreConditionConstraint> ProblemBuildingMasterPreConditionConstraint::create(alica::ConstraintContext& context)
{
    return std::make_unique<ProblemBuildingMasterPreConditionConstraint>();
}

} // namespace alica
