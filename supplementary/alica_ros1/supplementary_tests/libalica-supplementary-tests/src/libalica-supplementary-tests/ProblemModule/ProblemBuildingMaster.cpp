#include <libalica-supplementary-tests/ProblemModule/ProblemBuildingMaster.h>

namespace alica
{
ProblemBuildingMaster::ProblemBuildingMaster(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> ProblemBuildingMasterUtilityFunction::getUtilityFunction(Plan* plan)
{
    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::unique_ptr<ProblemBuildingMaster> ProblemBuildingMaster::create(alica::PlanContext& context)
{
    return std::make_unique<ProblemBuildingMaster>(context);
}

std::shared_ptr<ProblemBuildingMasterUtilityFunction> ProblemBuildingMasterUtilityFunction::create(alica::UtilityFunctionContext& context)
{
    return std::make_shared<ProblemBuildingMasterUtilityFunction>();
}

} // namespace alica
