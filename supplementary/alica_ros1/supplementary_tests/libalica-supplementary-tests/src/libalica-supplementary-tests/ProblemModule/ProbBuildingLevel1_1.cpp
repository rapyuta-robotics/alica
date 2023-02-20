#include <libalica-supplementary-tests/ProblemModule/ProbBuildingLevel1_1.h>

namespace alica
{
ProbBuildingLevel1_1::ProbBuildingLevel1_1(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> ProbBuildingLevel1_1UtilityFunction::getUtilityFunction(Plan* plan)
{
    std::shared_ptr<UtilityFunction> defaultFunction = std::make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::unique_ptr<ProbBuildingLevel1_1> ProbBuildingLevel1_1::create(alica::PlanContext& context)
{
    return std::make_unique<ProbBuildingLevel1_1>(context);
}

std::unique_ptr<ProbBuildingLevel1_1UtilityFunction> ProbBuildingLevel1_1UtilityFunction::create(alica::UtilityFunctionContext& context)
{
    return std::make_unique<ProbBuildingLevel1_1UtilityFunction>();
}

} // namespace alica
