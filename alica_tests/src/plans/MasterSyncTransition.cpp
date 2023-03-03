#include <alica_tests/TestWorldModel.h>
#include <alica_tests/plans/MasterSyncTransition.h>
#include <engine/AlicaEngine.h>

namespace alica
{
MasterSyncTransition::MasterSyncTransition(PlanContext& context)
        : BasicPlan(context)
{
}
} // namespace alica
