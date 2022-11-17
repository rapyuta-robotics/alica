#include "AcmePlan.h"

namespace alica
{

AcmePlan::AcmePlan(PlanContext& context)
        : BasicPlan(context)
{
    std::cerr << "AcmePlan created" << std::endl;
}
} // namespace alica
