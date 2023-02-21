#include <libalica-supplementary-tests/ProblemModule/QueryPlan2.h>

namespace alica
{
QueryPlan2::QueryPlan2(PlanContext& context)
        : BasicPlan(context)
{
}

std::shared_ptr<UtilityFunction> QueryPlan2UtilityFunction::getUtilityFunction(Plan* plan)
{
    shared_ptr<UtilityFunction> defaultFunction = make_shared<DefaultUtilityFunction>(plan);
    return defaultFunction;
}

std::unique_ptr<QueryPlan2> QueryPlan2::create(alica::PlanContext& context)
{
    return std::make_unique<QueryPlan2>(context);
}

std::unique_ptr<QueryPlan2UtilityFunction> QueryPlan2UtilityFunction::create(alica::UtilityFunctionContext& context)
{
    return std::make_unique<QueryPlan2UtilityFunction>();
}

} // namespace alica
