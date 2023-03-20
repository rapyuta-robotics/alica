#include "engine/modelmanagement/factories/VariableBindingFactory.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{
VariableBinding* VariableBindingFactory::create(const YAML::Node& node)
{
    auto* variableBinding = new VariableBinding();
    Factory::setAttributes(node, variableBinding);
    Factory::storeElement(variableBinding, alica::Strings::variableBinding);

    if (Factory::isValid(node[alica::Strings::variable])) {
        Factory::bindingVarReferences.push_back(
                std::pair<int64_t, int64_t>(variableBinding->getId(), Factory::getReferencedId(node[alica::Strings::variable])));
    }

    if (Factory::isValid(node[alica::Strings::subPlan])) {
        Factory::bindingSubPlanReferences.push_back(
                std::pair<int64_t, int64_t>(variableBinding->getId(), Factory::getReferencedId(node[alica::Strings::subPlan])));
    }

    if (Factory::isValid(node[alica::Strings::subVariable])) {
        Factory::bindingSubVarReferences.push_back(
                std::pair<int64_t, int64_t>(variableBinding->getId(), Factory::getReferencedId(node[alica::Strings::subVariable])));
    }

    return variableBinding;
}

void VariableBindingFactory::attachReferences()
{
    for (std::pair<int64_t, int64_t> pairs : Factory::bindingSubPlanReferences) {
        VariableBinding* p = (VariableBinding*) Factory::getElement(pairs.first);
        AbstractPlan* ap = (AbstractPlan*) Factory::getElement(pairs.second);
        p->setSubPlan(ap);
    }
    Factory::bindingSubPlanReferences.clear();

    for (std::pair<int64_t, int64_t> pairs : Factory::bindingSubVarReferences) {
        VariableBinding* p = (VariableBinding*) Factory::getElement(pairs.first);
        Variable* ap = (Variable*) Factory::getElement(pairs.second);
        p->setSubVar(ap);
    }
    Factory::bindingSubVarReferences.clear();

    for (std::pair<int64_t, int64_t> pairs : Factory::bindingVarReferences) {
        VariableBinding* p = (VariableBinding*) Factory::getElement(pairs.first);
        Variable* v = (Variable*) Factory::getElement(pairs.second);
        p->setVar(v);
    }
    Factory::bindingVarReferences.clear();
}
} // namespace alica
