#include <libalica-supplementary-tests/ProblemModule/ProbBuildingLevel1.h>

namespace alica
{
ProbBuildingLevel1::ProbBuildingLevel1(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> ProbBuildingLevel1UtilityFunction::getUtilityFunction(Plan* plan)
{
    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::unique_ptr<ProbBuildingLevel1> ProbBuildingLevel1::create(alica::PlanContext& context)
{
    return std::make_unique<ProbBuildingLevel1>(context);
}

std::unique_ptr<ProbBuildingLevel1UtilityFunction> ProbBuildingLevel1UtilityFunction::create(alica::UtilityFunctionContext& context)
{
    return std::make_unique<ProbBuildingLevel1UtilityFunction>();
}

} // namespace alica
