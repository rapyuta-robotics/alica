#include "engine/modelmanagement/factories/ConfAbstractPlanWrapperFactory.h"

#include "engine/model/ConfAbstractPlanWrapper.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{
ConfAbstractPlanWrapper* ConfAbstractPlanWrapperFactory::create(const YAML::Node& wrapperNode)
{
    ConfAbstractPlanWrapper* wrapper = new ConfAbstractPlanWrapper();
    Factory::setAttributes(wrapperNode, wrapper);
    Factory::storeElement(wrapper, alica::Strings::confAbstractPlanWrapper);

    if (Factory::isValid(wrapperNode[alica::Strings::abstractPlan])) {
        Factory::wrapperAbstractPlanReferences.push_back(
                std::pair<int64_t, int64_t>(wrapper->getId(), Factory::getReferencedId(wrapperNode[alica::Strings::abstractPlan])));
    }

    if (Factory::isValid(wrapperNode[alica::Strings::configuration])) {
        Factory::wrapperConfigurationReferences.push_back(
                std::pair<int64_t, int64_t>(wrapper->getId(), Factory::getReferencedId(wrapperNode[alica::Strings::configuration])));
    }

    return wrapper;
}

void ConfAbstractPlanWrapperFactory::attachReferences()
{
    // wrapperAbstractPlan references
    for (std::pair<int64_t, int64_t> pairs : Factory::wrapperAbstractPlanReferences) {
        ConfAbstractPlanWrapper* wrapper = (ConfAbstractPlanWrapper*) Factory::getElement(pairs.first);
        AbstractPlan* abstractPlan = (AbstractPlan*) Factory::getElement(pairs.second);
        wrapper->_abstractPlan = abstractPlan;
    }
    Factory::wrapperAbstractPlanReferences.clear();

    // wrapperConfiguration references
    for (std::pair<int64_t, int64_t> pairs : Factory::wrapperConfigurationReferences) {
        ConfAbstractPlanWrapper* wrapper = (ConfAbstractPlanWrapper*) Factory::getElement(pairs.first);
        Configuration* configuration = (Configuration*) Factory::getElement(pairs.second);
        wrapper->_configuration = configuration;
    }
    Factory::wrapperConfigurationReferences.clear();
}
} // namespace alica
