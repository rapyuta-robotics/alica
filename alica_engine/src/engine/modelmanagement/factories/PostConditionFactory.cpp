#include "engine/modelmanagement/factories/PostConditionFactory.h"
#include "engine/model/PostCondition.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/ConditionFactory.h"

namespace alica
{
PostCondition* PostConditionFactory::create(const YAML::Node& postConditionNode, AbstractPlan* plan)
{
    PostCondition* postCondition = new PostCondition();
    ConditionFactory::fillCondition(postConditionNode, postCondition, plan);
    return postCondition;
}
} // namespace alica
