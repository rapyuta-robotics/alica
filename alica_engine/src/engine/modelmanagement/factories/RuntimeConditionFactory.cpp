#include "engine/modelmanagement/factories/RuntimeConditionFactory.h"
#include "engine/modelmanagement/factories/ConditionFactory.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/model/RuntimeCondition.h"

namespace alica
{
    RuntimeCondition* RuntimeConditionFactory::create(const YAML::Node& runtimeConditionNode, AbstractPlan* plan)
    {
        RuntimeCondition* runtimeCondition = new RuntimeCondition();
        ConditionFactory::fillCondition(runtimeConditionNode, runtimeCondition, plan);
        return runtimeCondition;
    }
} // namespace alica
