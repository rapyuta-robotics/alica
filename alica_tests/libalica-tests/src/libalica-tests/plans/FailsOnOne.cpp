#include <alica_tests/SimpleSwitches.h>
#include <libalica-tests/plans/FailsOnOne.h>

namespace alica
{
FailsOnOne::FailsOnOne(PlanContext& context)
        : BasicPlan(context)
{
}
bool FailsOnOneRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return !SimpleSwitches::isSet(1);
}
} // namespace alica
