#include "engine/modelmanagement/factories/AbstractPlanFactory.h"
#include "engine/model/AbstractPlan.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/VariableFactory.h"

namespace alica
{
void AbstractPlanFactory::setVariables(const YAML::Node& abstractPlanNode, alica::AbstractPlan* abstractPlan)
{
    if (Factory::isValid(abstractPlanNode[alica::Strings::variables])) {
        const YAML::Node& variableNodes = abstractPlanNode[alica::Strings::variables];
        for (YAML::const_iterator it = variableNodes.begin(); it != variableNodes.end(); ++it) {
            abstractPlan->addVariable(VariableFactory::create(*it));
        }
    }
}
} // namespace alica
