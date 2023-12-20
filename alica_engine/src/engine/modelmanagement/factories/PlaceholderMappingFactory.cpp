#include "engine/modelmanagement/factories/PlaceholderMappingFactory.h"

#include "engine/blackboard/Blackboard.h"
#include "engine/model/AbstractPlan.h"
#include "engine/model/Behaviour.h"
#include "engine/model/Placeholder.h"
#include "engine/model/PlaceholderMapping.h"
#include "engine/model/Plan.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{

PlaceholderMapping* PlaceholderMappingFactory::create(const YAML::Node& node)
{
    PlaceholderMapping* mapping = new PlaceholderMapping();
    setAttributes(node, mapping);
    storeElement(mapping, alica::Strings::placeholderMapping);

    YAML::Node mappingNode = node[alica::Strings::placeholderMapping];

    for (const YAML::Node& entry : mappingNode) {
        placeholderReferences.push_back({mapping->getId(), getReferencedId(entry[alica::Strings::placeholder])});
        placeholderAbstractPlanReferences.push_back({mapping->getId(), getReferencedId(entry[alica::Strings::placeholderImplementation])});
    }

    return mapping;
}

void PlaceholderMappingFactory::attachReferences()
{
    assert(placeholderReferences.size() == placeholderAbstractPlanReferences.size());
    for (auto placeholderIt = placeholderReferences.begin(), abstractPlanIt = placeholderAbstractPlanReferences.begin();
            placeholderIt != placeholderReferences.end(); ++placeholderIt, ++abstractPlanIt) {
        assert(placeholderIt->first == abstractPlanIt->first);

        auto mapping = dynamic_cast<PlaceholderMapping*>(const_cast<AlicaElement*>(getElement(placeholderIt->first)));
        assert(mapping);
        auto abstractPlan = dynamic_cast<const AbstractPlan*>(getElement(abstractPlanIt->second));
        assert(abstractPlan);
        auto placeholder = dynamic_cast<const Placeholder*>(getPlaceholder(placeholderIt->second));
        assert(placeholder);

        // for behaviors / plans which replace placeholders, the blackboard blueprint has to be the equal
        if (const Behaviour* beh = dynamic_cast<const Behaviour*>(abstractPlan)) {
            if (*placeholder->getBlackboardBlueprint() != *beh->getBlackboardBlueprint()) {
                AlicaEngine::abort(LOGNAME, "Invalid implementation found for the placeholder '", placeholder->getName(), "'");
            }
        } else if (const Plan* plan = dynamic_cast<const Plan*>(abstractPlan)) {
            if (*placeholder->getBlackboardBlueprint() != *plan->getBlackboardBlueprint()) {
                AlicaEngine::abort(LOGNAME, "Invalid implementation found for the placeholder '", placeholder->getName(), "'");
            }
        }
        mapping->_mapping.emplace(placeholderIt->second, abstractPlan);
    }
}

} // namespace alica
