#include "engine/modelmanagement/factories/PlanTypeFactory.h"

#include "engine/model/PlanType.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/AbstractPlanFactory.h"

#include <vector>

namespace alica
{
    PlanType* PlanTypeFactory::create(const YAML::Node& planTypeNode)
    {
        PlanType* planType = new PlanType();
        Factory::setAttributes(planTypeNode, planType);
        Factory::storeElement(planType, alica::Strings::plantype);
        AbstractPlanFactory::setVariables(planTypeNode, planType);

        if (Factory::isValid(planTypeNode[alica::Strings::annotatedPlans])) {
            const YAML::Node &annotatedPlanNodes = planTypeNode[alica::Strings::annotatedPlans];
            for (YAML::const_iterator it = annotatedPlanNodes.begin(); it != annotatedPlanNodes.end(); ++it) {
                // only add plans that are annotated to be active
                if (Factory::getValue<bool>(*it, alica::Strings::activated, false)) {
                    const std::string& referencedPlanString = Factory::getValue<std::string>(*it, alica::Strings::plan);
                    Factory::planTypePlanReferences.push_back(std::pair<int64_t, int64_t>(planType->getId(), Factory::getReferencedId(referencedPlanString)));
                }
            }
        }

        return planType;
    }
} // namespace alica