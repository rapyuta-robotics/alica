#include "engine/modelmanagement/factories/RuntimeConditionFactory.h"
#include "engine/model/RuntimeCondition.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/ConditionFactory.h"

namespace alica
{
RuntimeCondition* RuntimeConditionFactory::create(const YAML::Node& runtimeConditionNode, AbstractPlan* plan)
{
    RuntimeCondition* runtimeCondition = new RuntimeCondition();
    ConditionFactory::fillCondition(runtimeConditionNode, runtimeCondition, plan);
    return runtimeCondition;
}
} // namespace alica
