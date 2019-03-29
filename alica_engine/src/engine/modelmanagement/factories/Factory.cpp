#include "engine/modelmanagement/factories/Factory.h"

#include "engine/model/AlicaElement.h"
#include "engine/modelmanagement/Strings.h"

namespace alica
{

ReferenceList Factory::stateInTransitionReferences;
ReferenceList Factory::stateOutTransitionReferences;
ReferenceList Factory::statePlanReferences;
ReferenceList Factory::transitionSynchReferences;
ReferenceList Factory::transitionInStateReferences;
ReferenceList Factory::transitionOutStateReferences;
ReferenceList Factory::paramSubPlanReferences;
ReferenceList Factory::paramSubVarReferences;
ReferenceList Factory::paramVarReferences;
ReferenceList Factory::conditionVarReferences;
ReferenceList Factory::quantifierScopeReferences;
ReferenceList Factory::epStateReferences;
ReferenceList Factory::epTaskReferences;
ReferenceList Factory::planTypePlanReferences;
ReferenceList Factory::rtmRoleReferences;
ReferenceList Factory::charCapReferences;
ReferenceList Factory::charCapValReferences;
PlanRepository* Factory::planRepository;
std::map<int64_t, AlicaElement*>* Factory::elements;

int64_t Factory::getReferencedId(const YAML::Node& node)
{
    std::cout << node << std::endl;
    std::string idString = node.as<std::string>();
    std::size_t idxOfHashtag = idString.find_last_of("#");
    if (idxOfHashtag == std::string::npos) {
        return node.as<int64_t>();
    } else {
        return stol(idString.substr(idxOfHashtag + 1, idString.size() - idxOfHashtag));
    }
}

void Factory::setElementsMap(std::map<int64_t, AlicaElement*>& elements)
{
    Factory::elements = &elements;
}

void Factory::setPlanRepository(PlanRepository* planRepository)
{
    Factory::planRepository = planRepository;
}

void Factory::setAttributes(const YAML::Node& node, alica::AlicaElement* ael)
{
    ael->_id = getValue<int64_t>(node, alica::Strings::id);
    ael->_name = getValue<std::string>(node, alica::Strings::name, alica::Strings::no_name);
    ael->_comment = getValue<std::string>(node, alica::Strings::comment, alica::Strings::no_comment);
}

const AlicaElement* Factory::getElement(const int64_t id)
{
    auto mapEntry = Factory::elements->find(id);
    if (mapEntry != Factory::elements->end()) {
        return mapEntry->second;
    }
    return nullptr;
}

void Factory::storeElement(AlicaElement* ael, const std::string& type)
{
    // insert into general element map
    if (elements->find(ael->getId()) != elements->end()) {
        std::stringstream ss;
        ss << "Factory: ERROR: ID utilised twice: " << ael->getId() << std::endl;
        ss << "ELEMENT >" << ael->getName() << "< >" << (*elements)[ael->getId()]->getName() << "<" << std::endl;
        AlicaEngine::abort(ss.str());
    }
    elements->insert(std::pair<int64_t, AlicaElement*>(ael->getId(), ael));

    // insert into plan repository
    if (alica::Strings::plan.compare(type) == 0) {
        planRepository->_plans.emplace(ael->getId(), (Plan*) ael);
    } else if (alica::Strings::entryPoint.compare(type) == 0) {
        planRepository->_entryPoints.emplace(ael->getId(), (EntryPoint*) ael);
    } else if (alica::Strings::state.compare(type) == 0) {
        planRepository->_states.emplace(ael->getId(), (State*) ael);
    } else if (alica::Strings::transition.compare(type) == 0) {
        planRepository->_transitions.emplace(ael->getId(), (Transition*) ael);
    } else if (alica::Strings::condition.compare(type) == 0) {
        planRepository->_conditions.emplace(ael->getId(), (Condition*) ael);
    } else if (alica::Strings::variable.compare(type) == 0) {
        planRepository->_variables.emplace(ael->getId(), (Variable*) ael);
    } else if (alica::Strings::quantifier.compare(type) == 0) {
        planRepository->_quantifiers.emplace(ael->getId(), (Quantifier*) ael);
    } else if (alica::Strings::synchronisation.compare(type) == 0) {
        planRepository->_synchronisations.emplace(ael->getId(), (Synchronisation*) ael);
    } else {
        AlicaEngine::abort("Factory: Element type unhandled for storing!");
    }
}

} // namespace alica