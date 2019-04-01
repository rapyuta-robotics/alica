#include "engine/modelmanagement/factories/Factory.h"

#include "engine/model/AlicaElement.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/PlanRepository.h"

namespace alica
{

ReferenceList Factory::stateInTransitionReferences;
ReferenceList Factory::stateOutTransitionReferences;
ReferenceList Factory::stateAbstractPlanReferences;
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
ModelManager* Factory::modelManager;

int64_t Factory::getReferencedId(const YAML::Node &node) {
    const std::string idString = node.as<std::string>();
    return Factory::getReferencedId(idString);
}

int64_t Factory::getReferencedId(const std::string& idString)
{
    std::size_t idxOfHashtag = idString.find_last_of("#");
    if (idxOfHashtag == std::string::npos) {
        return stoll(idString);
    }
    std::string locator = idString.substr(0, idxOfHashtag);
    if (!locator.empty()) {
        std::string fileReferenced;
        if (essentials::FileSystem::endsWith(locator, ".pml")
            || essentials::FileSystem::endsWith(locator, ".beh")
            || essentials::FileSystem::endsWith(locator, ".pty")) {
            fileReferenced = essentials::FileSystem::combinePaths(modelManager->basePlanPath, locator);
        } else if (essentials::FileSystem::endsWith(locator, ".tsk")) {
            fileReferenced = essentials::FileSystem::combinePaths(modelManager->baseTaskPath, locator);
        } else if (essentials::FileSystem::endsWith(locator, ".rst")) {
            fileReferenced = essentials::FileSystem::combinePaths(modelManager->baseRolePath, locator);
        } else {
            std::cout << "Factory: Unknown file extension: " << locator << std::endl;
        }

        if (std::find(std::begin(modelManager->filesParsed), std::end(modelManager->filesParsed), fileReferenced) == std::end(modelManager->filesParsed) &&
            std::find(std::begin(modelManager->filesToParse), std::end(modelManager->filesToParse), fileReferenced) == std::end(modelManager->filesToParse)) {
            modelManager->filesToParse.push_back(fileReferenced);
        }
    }
    return stoll(idString.substr(idxOfHashtag + 1, idString.size() - idxOfHashtag));
}

std::vector<int64_t> Factory::getReferencedIds(const YAML::Node& referenceListNode) {
    std::vector<int64_t> ids;

    // TODO not working, yet - maybe adapt jackson mixin, that would be simpler...
    std::string references = referenceListNode.as<std::string>();
    size_t startIdx = 0;
    size_t endIdx = references.find_first_of(",", startIdx);
    while (endIdx != std::string::npos) {
        ids.push_back(Factory::getReferencedId(references.substr(startIdx, endIdx-startIdx)));
        startIdx = endIdx+1;
        endIdx = references.find_first_of(",", startIdx);
    }
    ids.push_back(Factory::getReferencedId(references.substr(startIdx, endIdx-startIdx)));

    return ids;
}

void Factory::setModelManager(alica::ModelManager* modelManager) {
    Factory::modelManager = modelManager;
}

void Factory::setAttributes(const YAML::Node& node, alica::AlicaElement* ael)
{
    ael->_id = getValue<int64_t>(node, alica::Strings::id);
    ael->_name = getValue<std::string>(node, alica::Strings::name, alica::Strings::no_name);
    ael->_comment = getValue<std::string>(node, alica::Strings::comment, alica::Strings::no_comment);
}

const AlicaElement* Factory::getElement(const int64_t id)
{
    auto mapEntry = Factory::modelManager->elements.find(id);
    if (mapEntry != Factory::modelManager->elements.end()) {
        return mapEntry->second;
    }
    return nullptr;
}

void Factory::storeElement(AlicaElement* ael, const std::string& type)
{
    // insert into general element map
    if (modelManager->elements.find(ael->getId()) != modelManager->elements.end()) {
        std::stringstream ss;
        ss << "Factory: ERROR: ID utilised twice: " << ael->getId() << std::endl;
        ss << "ELEMENT >" << ael->getName() << "< >" << modelManager->elements[ael->getId()]->getName() << "<" << std::endl;
        AlicaEngine::abort(ss.str());
    }
    modelManager->elements.insert(std::pair<int64_t, AlicaElement*>(ael->getId(), ael));

    // insert into plan repository
    if (alica::Strings::plan.compare(type) == 0) {
        modelManager->planRepository->_plans.emplace(ael->getId(), (Plan*) ael);
    } else if (alica::Strings::entryPoint.compare(type) == 0) {
        modelManager->planRepository->_entryPoints.emplace(ael->getId(), (EntryPoint*) ael);
    } else if (alica::Strings::state.compare(type) == 0) {
        modelManager->planRepository->_states.emplace(ael->getId(), (State*) ael);
    } else if (alica::Strings::transition.compare(type) == 0) {
        modelManager->planRepository->_transitions.emplace(ael->getId(), (Transition*) ael);
    } else if (alica::Strings::condition.compare(type) == 0) {
        modelManager->planRepository->_conditions.emplace(ael->getId(), (Condition*) ael);
    } else if (alica::Strings::variable.compare(type) == 0) {
        modelManager->planRepository->_variables.emplace(ael->getId(), (Variable*) ael);
    } else if (alica::Strings::quantifier.compare(type) == 0) {
        modelManager->planRepository->_quantifiers.emplace(ael->getId(), (Quantifier*) ael);
    } else if (alica::Strings::synchronisation.compare(type) == 0) {
        modelManager->planRepository->_synchronisations.emplace(ael->getId(), (Synchronisation*) ael);
    } else if (alica::Strings::behaviour.compare(type) == 0) {
        modelManager->planRepository->_behaviours.emplace(ael->getId(), (Behaviour*) ael);
    } else if (alica::Strings::task.compare(type) == 0) {
        modelManager->planRepository->_tasks.emplace(ael->getId(), (Task*) ael);
    } else if (alica::Strings::taskrepository.compare(type) == 0) {
        modelManager->planRepository->_taskRepositories.emplace(ael->getId(), (TaskRepository*) ael);
    } else if (alica::Strings::plantype.compare(type) == 0) {
        modelManager->planRepository->_planTypes.emplace(ael->getId(), (PlanType*) ael);
    } else {
        AlicaEngine::abort("Factory: Element type unhandled for storing!");
    }
}

} // namespace alica