#include "engine/RunningPlan.h"
#include <alica_tests/TestWorldModel.h>
#include <alica_tests/plans/SchedulingTestSequenceSubPlan1.h>

namespace alica
{
SchedulingTestSequenceSubPlan1::SchedulingTestSequenceSubPlan1(PlanContext& context)
        : BasicPlan(context)
{
}
} // namespace alica
