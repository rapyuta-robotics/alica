#include "engine/modelmanagement/factories/PlaceholderMappingFactory.h"

#include "engine/model/PlaceholderMapping.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{

PlaceholderMapping* PlaceholderMappingFactory::create(const YAML::Node& node)
{
    PlaceholderMapping* mapping = new PlaceholderMapping();
    setAttributes(node, mapping);
    storeElement(mapping, alica::Strings::placeholderMapping);

    YAML::Node mappingNode = node[alica::Strings::placeholderMapping];
    for (YAML::Node entry : mappingNode) {
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
        // TODO: verify that the placeholder & abstract plans are compatible
        mapping->_mapping.emplace(placeholderIt->second, dynamic_cast<const AbstractPlan*>(getElement(abstractPlanIt->second)));
    }
}

} // namespace alica
