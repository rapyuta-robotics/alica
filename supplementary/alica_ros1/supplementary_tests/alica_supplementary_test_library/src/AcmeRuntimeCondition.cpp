#include "AcmeRuntimeCondition.h"

namespace alica
{

AcmeRuntimeCondition::AcmeRuntimeCondition()
{
    std::cerr << "Debug:"
              << "AcmeRuntimeCondition created" << std::endl;
}

bool AcmeRuntimeCondition::evaluate(std::shared_ptr<RunningPlan> rp, const Blackboard* globalBlackboard)
{
    std::cerr << "AcmeRuntimeCondition::evaluate" << std::endl;
    return true;
}

} // namespace alica
