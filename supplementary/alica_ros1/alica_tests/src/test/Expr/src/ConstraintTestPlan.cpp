#include <alica/test/CounterClass.h>
#include <alica_tests/ConstraintTestPlan.h>

namespace alica
{
ConstraintTestPlan::ConstraintTestPlan(PlanContext& context)
        : DomainPlan(context)
{
}
bool ConstraintTestPlanRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* gb)
{
    return true;
}

void ConstraintTestPlanRuntimeConditionConstraint::getConstraint(std::shared_ptr<ProblemDescriptor> c, std::shared_ptr<RunningPlan> rp)
{
    std::cout << "#########################################" << std::endl;
    CounterClass::called++;
}
} // namespace alica