#include <libalica-supplementary-tests/GSolver/GSolverMaster.h>

namespace alica
{
GSolverMaster::GSolverMaster(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> GSolverMasterUtilityFunction::getUtilityFunction(Plan* plan)
{
    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::unique_ptr<GSolverMaster> GSolverMaster::create(PlanContext& context)
{
    return std::make_unique<GSolverMaster>(context);
}

std::shared_ptr<GSolverMasterUtilityFunction> GSolverMasterUtilityFunction::create(UtilityFunctionContext& context)
{
    return std::make_shared<GSolverMasterUtilityFunction>();
}

} // namespace alica
