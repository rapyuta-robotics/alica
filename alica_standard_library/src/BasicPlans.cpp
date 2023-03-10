#include "BasicPlans.h"

namespace alica_standard_library
{

TracedPlan::TracedPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
    setTracing(TracingType::DEFAULT);
}

std::unique_ptr<TracedPlan> TracedPlan::create(alica::PlanContext& context)
{
    return std::make_unique<TracedPlan>(context);
}

UntracedPlan::UntracedPlan(alica::PlanContext& context)
        : BasicPlan(context)
{
    setTracing(TracingType::SKIP);
}

std::unique_ptr<UntracedPlan> UntracedPlan::create(alica::PlanContext& context)
{
    return std::make_unique<UntracedPlan>(context);
}

} // namespace alica_standard_library
