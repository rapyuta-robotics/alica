#include <alica_tests/FailsOnOne.h>
#include <alica_tests/SimpleSwitches.h>

namespace alica
{
FailsOnOne::FailsOnOne(PlanContext& context)
        : DomainPlan(context)
{
}
bool RunTimeCondition1530069251117::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return !SimpleSwitches::isSet(1);
}
} // namespace alica
