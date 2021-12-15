#include "engine/modelmanagement/factories/PlanTypeFactory.h"

#include "engine/model/PlanType.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/AbstractPlanFactory.h"
#include "engine/modelmanagement/factories/VariableBindingFactory.h"

#include <vector>

namespace alica
{
PlanType* PlanTypeFactory::create(AlicaEngine* ae, const YAML::Node& planTypeNode)
{
    PlanType* planType = new PlanType(ae);
    Factory::setAttributes(planTypeNode, planType);
    Factory::storeElement(planType, alica::Strings::plantype);
    AbstractPlanFactory::setVariables(planTypeNode, planType);

    if (Factory::isValid(planTypeNode[alica::Strings::annotatedPlans])) {
        const YAML::Node& annotatedPlanNodes = planTypeNode[alica::Strings::annotatedPlans];
        for (YAML::const_iterator it = annotatedPlanNodes.begin(); it != annotatedPlanNodes.end(); ++it) {
            // only add plans that are annotated to be active
            if (Factory::getValue<bool>(*it, alica::Strings::activated, false)) {
                const std::string& referencedPlanString = Factory::getValue<std::string>(*it, alica::Strings::plan);
                Factory::planTypePlanReferences.push_back(std::pair<int64_t, int64_t>(planType->getId(), Factory::getReferencedId(referencedPlanString)));
            }
        }
    }

    if (Factory::isValid(planTypeNode[alica::Strings::variableBindings])) {
        const YAML::Node& variableBindings = planTypeNode[alica::Strings::variableBindings];
        for (YAML::const_iterator it = variableBindings.begin(); it != variableBindings.end(); ++it) {
            planType->_variableBindings.push_back(VariableBindingFactory::create(*it));
        }
    }

    return planType;
}

void PlanTypeFactory::attachReferences()
{
    VariableBindingFactory::attachReferences();

    // planTypePlanReferences
    for (std::pair<int64_t, int64_t> pairs : Factory::planTypePlanReferences) {
        PlanType* pt = (PlanType*) Factory::getElement(pairs.first);
        Plan* p = (Plan*) Factory::getElement(pairs.second);
        pt->_plans.push_back(p);
    }
    Factory::planTypePlanReferences.clear();
}
} // namespace alica