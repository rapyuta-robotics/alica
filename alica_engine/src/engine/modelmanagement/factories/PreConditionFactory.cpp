#include "engine/modelmanagement/factories/PreConditionFactory.h"
#include "engine/modelmanagement/factories/ConditionFactory.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/model/PreCondition.h"

namespace alica
{
    PreCondition* PreConditionFactory::create(const YAML::Node& preConditionNode, AbstractPlan* plan)
    {
        PreCondition* preCondition = new PreCondition();
        ConditionFactory::fillCondition(preConditionNode, preCondition, plan);
        preCondition->_enabled = Factory::getValue<bool>(preConditionNode, alica::Strings::enabled);
        return preCondition;
    }
} // namespace alica
