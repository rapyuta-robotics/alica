#include <alica_tests/TestWorldModel.h>
#include <engine/AlicaEngine.h>
#include <libalica-tests/plans/MasterSyncTransition.h>

namespace alica
{
MasterSyncTransition::MasterSyncTransition(PlanContext& context)
        : BasicPlan(context)
{
}
} // namespace alica
