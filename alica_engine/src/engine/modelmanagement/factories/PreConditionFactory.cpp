#include "engine/modelmanagement/factories/PreConditionFactory.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/model/PreCondition.h"

namespace alica
{
    PreCondition* PreConditionFactory::create(const YAML::Node& preConditionNode, Plan* plan)
    {
        PreCondition* preCondition = new PreCondition();
        Factory::setAttributes(preConditionNode, preCondition);
        Factory::storeElement(preCondition, alica::Strings::precondition);

        preCondition->setAbstractPlan((AbstractPlan*) plan);
        preCondition->_enabled = Factory::getValue<bool>(preConditionNode, alica::Strings::enabled);
        preCondition->_conditionString = Factory::getValue<std::string>(preConditionNode, alica::Strings::conditionString);
        preCondition->_plugInName = Factory::getValue<std::string>(preConditionNode, alica::Strings::pluginName);
        if (preConditionNode[alica::Strings::variables]) {

        }
        if (preConditionNode[alica::Strings::quantifiers]) {

        }

        return preCondition;
    }
} // namespace alica
