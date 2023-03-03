#include <alica_tests/MasterSyncTransition.h>
#include <alica_tests/TestWorldModel.h>
#include <engine/AlicaEngine.h>

namespace alica
{
MasterSyncTransition::MasterSyncTransition(PlanContext& context)
        : DomainPlan(context)
{
}
} // namespace alica
