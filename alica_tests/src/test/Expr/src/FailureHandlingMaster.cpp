#include <alica_tests/TestWorldModel.h>
#include <libalica-tests/plans/FailureHandlingMaster.h>

namespace alica
{
FailureHandlingMaster::FailureHandlingMaster(PlanContext& context)
        : BasicPlan(context)
{
}
} // namespace alica
