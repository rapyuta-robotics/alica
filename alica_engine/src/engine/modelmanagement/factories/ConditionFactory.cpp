#include "engine/modelmanagement/factories/ConditionFactory.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Condition.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/QuantifierFactory.h"

namespace alica
{
void ConditionFactory::fillCondition(const YAML::Node& conditionNode, Condition* condition, alica::AbstractPlan* abstractPlan)
{
    Factory::setAttributes(conditionNode, condition);
    Factory::storeElement(condition, alica::Strings::condition);
    condition->_abstractPlan = abstractPlan;
    condition->_conditionString = Factory::getValue<std::string>(conditionNode, alica::Strings::conditionString, "");
    condition->_plugInName = Factory::getValue<std::string>(conditionNode, alica::Strings::pluginName);
    if (Factory::isValid(conditionNode[alica::Strings::variables])) {
        const YAML::Node& variableNodes = conditionNode[alica::Strings::variables];
        for (YAML::const_iterator it = variableNodes.begin(); it != variableNodes.end(); ++it) {
            Factory::conditionVarReferences.push_back(std::pair<int64_t, int64_t>(condition->getId(), Factory::getReferencedId(*it)));
        }
    }
    if (Factory::isValid(conditionNode[alica::Strings::quantifiers])) {
        const YAML::Node& quantifierNodes = conditionNode[alica::Strings::quantifiers];
        for (YAML::const_iterator it = quantifierNodes.begin(); it != quantifierNodes.end(); ++it) {
            condition->_quantifiers.push_back(QuantifierFactory::create(*it));
        }
    }
}

void ConditionFactory::attachReferences()
{
    QuantifierFactory::attachReferences();

    // conditionVarReferences
    for (std::pair<int64_t, int64_t> pairs : Factory::conditionVarReferences) {
        Condition* c = (Condition*) Factory::getElement(pairs.first);
        Variable* v = (Variable*) Factory::getElement(pairs.second);
        c->_variables.push_back(v);
    }
    Factory::conditionVarReferences.clear();
}
} // namespace alica
