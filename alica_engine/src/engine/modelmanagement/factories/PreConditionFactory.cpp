#include "engine/modelmanagement/factories/PreConditionFactory.h"
#include "engine/model/PreCondition.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/ConditionFactory.h"

namespace alica
{
PreCondition* PreConditionFactory::create(const YAML::Node& preConditionNode, AbstractPlan* plan)
{
    auto* preCondition = new PreCondition();
    ConditionFactory::fillCondition(preConditionNode, preCondition, plan);
    preCondition->setEnabled(Factory::getValue<bool>(preConditionNode, alica::Strings::enabled));
    return preCondition;
}
} // namespace alica
