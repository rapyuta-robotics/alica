#include "engine/modelmanagement/ModelManager.h"

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/modelmanagement/Strings.h"

#include <SystemConfig.h>
#include <alica_common_config/debug_output.h>

#include <engine/modelmanagement/ModelManager.h>
#include <sstream>

namespace alica
{

ModelManager::ModelManager(PlanRepository* planRepository)
{
    this->planRepository = planRepository;
    this->sc = essentials::SystemConfig::getInstance();
    this->domainConfigFolder = this->sc->getConfigPath();
    this->basePlanPath = getBasePath("Alica.PlanDir");
    this->baseRolePath = getBasePath("Alica.RoleDir");
    this->baseTaskPath = getBasePath("Alica.TaskDir");
}

const std::string ModelManager::getBasePath(const std::string& configKey)
{
    std::string basePath = "";
    try {
        basePath = (*this->sc)["Alica"]->get<std::string>(configKey.c_str(), NULL);
    } catch (const std::runtime_error& error) {
        AlicaEngine::abort("MM: Directory for config key " + configKey + " does not exist.\n", error.what());
    }

    if (!essentials::FileSystem::endsWith(basePath, essentials::FileSystem::PATH_SEPARATOR)) {
        basePath = basePath + essentials::FileSystem::PATH_SEPARATOR;
    }

    if (!essentials::FileSystem::isPathRooted(basePath)) {
        basePath = this->domainConfigFolder + basePath;
    }

    ALICA_DEBUG_MSG("MM: config key '" + configKey + "' maps to '" + basePath + "'");

    if (!essentials::FileSystem::pathExists(basePath)) {
        AlicaEngine::abort("MM: base path does not exist: " + basePlanPath);
    }
    return basePath;
}

Plan* ModelManager::loadPlanTree(const std::string& masterPlanName)
{
    std::string masterPlanPath;
    if (!essentials::FileSystem::findFile(this->basePlanPath, masterPlanName + alica::Strings::plan_extension, masterPlanPath)) {
        AlicaEngine::abort("MM: Cannot find MasterPlan '" + masterPlanName + "'");
    }

    //    this->filesParsed.push_back(masterPlanPath);
    Plan* masterPlan = (Plan*) parseFile(masterPlanPath, alica::Strings::plan);
    //    parseFileLoop();

    //    this->mf->computeReachabilities();
    return masterPlan;
}

RoleSet* ModelManager::loadRoleSet(const std::string& roleSetName)
{
    AlicaEngine::abort("MM: loadRoleSet not implemented, yet!");
    return nullptr;
}

AlicaElement* ModelManager::parseFile(const std::string& currentFile, const std::string& type)
{
    YAML::Node doc;
    try {
        doc = YAML::LoadFile(currentFile);
    } catch (YAML::BadFile badFile) {
        AlicaEngine::abort("MM: Could not parse file: ", badFile.msg);
    }

    if (alica::Strings::plan.compare(type) == 0) {
        Plan* plan = createPlan(doc);
        plan->setFileName(currentFile);
        return plan;
    } else {
        AlicaEngine::abort("MM: Parsing type not handled: ", type);
        return nullptr;
    }
}

Plan* ModelManager::createPlan(const YAML::Node& node)
{
    Plan* plan = new Plan(this->getId(node));
    storeElement(plan, alica::Strings::plan);
    plan->_name = this->getName(node);
    plan->_comment = this->getComment(node);

    if (node[alica::Strings::masterPlan]) {
        plan->_masterPlan = node[alica::Strings::masterPlan].as<bool>();
    }
    if (node[alica::Strings::utilityThreshold]) {
        plan->_utilityThreshold = node[alica::Strings::utilityThreshold].as<double>();
    }
    if (node[alica::Strings::entryPoints]) {
        this->createEntryPoints(node[alica::Strings::entryPoints], plan);
    }

    /*
     if (states.compare(val) == 0) {
        string name = "";
        const char* typePtr = curChild->Attribute("xsi:type");
        string typeString = "";
        // Normal State
        if (typePtr) {
            typeString = typePtr;
        }
        if (typeString.empty()) {
            State* state = createState(curChild);
            plan->_states.push_back(state);
            state->setInPlan(plan);

        } else if (typeString.compare("alica:SuccessState") == 0) {
            SuccessState* suc = createSuccessState(curChild);
            suc->setInPlan(plan);
            plan->_successStates.push_back(suc);
            plan->_states.push_back(suc);

        } else if (typeString.compare("alica:FailureState") == 0) {
            FailureState* fail = createFailureState(curChild);
            fail->setInPlan(plan);
            plan->_failureStates.push_back(fail);
            plan->_states.push_back(fail);
        } else {
            AlicaEngine::abort("MF: Unknown State type:", typePtr);
        }
    } else if (transitions.compare(val) == 0) {
        Transition* tran = createTransition(curChild, plan);
        plan->_transitions.push_back(tran);
    } else if (preCondition.compare(val) == 0) {
        PreCondition* p = createPreCondition(curChild);
        p->setAbstractPlan(plan);
        plan->setPreCondition(p);
    } else if (runtimeCondition.compare(val) == 0) {
        RuntimeCondition* rc = createRuntimeCondition(curChild);
        rc->setAbstractPlan(plan);
        plan->setRuntimeCondition(rc);
    } else if (postCondition.compare(val) == 0) {
        PostCondition* p = createPostCondition(curChild);
        plan->setPostCondition(p);
    } else if (vars.compare(val) == 0) {
        Variable* var = createVariable(curChild);
        plan->_variables.push_back(var);
    } else if (synchronisations.compare(val) == 0) {
        SyncTransition* st = createSyncTransition(curChild);
        st->setPlan(plan);
        plan->_syncTransitions.push_back(st);

    }
*/

    return plan;
}

void ModelManager::createEntryPoints(const YAML::Node& entryPoints, Plan* plan)
{
    std::vector<EntryPoint*> constructedEntryPoints;
    for (YAML::const_iterator it = entryPoints.begin(); it != entryPoints.end(); ++it) {
        const YAML::Node& epNode = *it;

        EntryPoint* ep = new EntryPoint();
        ep->_id = this->getId(epNode);
        this->storeElement(ep, alica::Strings::entryPoint);
        ep->_name = this->getName(epNode);
        ep->_comment = this->getComment(epNode);
        ep->_plan = (Plan*) this->getElement(this->getReferencedId(epNode[alica::Strings::plan]));
        if (epNode[alica::Strings::minCardinality]) {
            ep->_cardinality.setMin(epNode[alica::Strings::minCardinality].as<int>());
        }
        if (epNode[alica::Strings::maxCardinality]) {
            ep->_cardinality.setMax(epNode[alica::Strings::maxCardinality].as<int>());
        }
        if (epNode[alica::Strings::successRequired]) {
            ep->setSuccessRequired(epNode[alica::Strings::successRequired].as<bool>());
        }
        if (epNode[alica::Strings::state]) {
            this->epStateReferences.push_back(std::pair<int64_t, int64_t>(ep->getId(), this->getReferencedId(epNode[alica::Strings::state])));
        }
        if (epNode[alica::Strings::task]) {
            this->epTaskReferences.push_back(std::pair<int64_t, int64_t>(ep->getId(), this->getReferencedId(epNode[alica::Strings::task])));
        }
        constructedEntryPoints.push_back(ep);
    }

    // SORT EntryPoints
    std::sort(constructedEntryPoints.begin(), constructedEntryPoints.end(),
            [](const EntryPoint* ep1, const EntryPoint* ep2) { return ep1->getId() < ep2->getId(); });

    // set indices, add to plan, and summarize min/max cardinality
    int minCard = 0;
    int maxCard = 0;
    plan->_entryPoints.reserve(constructedEntryPoints.size());
    for (int i = 0; i < static_cast<int>(constructedEntryPoints.size()); ++i) {
        constructedEntryPoints[i]->_index = i;
        plan->_entryPoints.push_back(constructedEntryPoints[i]);
        minCard += constructedEntryPoints[i]->getCardinality().getMin();
        maxCard += constructedEntryPoints[i]->getCardinality().getMax();
    }
    plan->setMinCardinality(minCard);
    plan->setMaxCardinality(maxCard);
}

int64_t ModelManager::getId(const YAML::Node& node)
{
    if (node[alica::Strings::id]) {
        return node[alica::Strings::id].as<std::int64_t>();
    } else {
        AlicaEngine::abort("MM: Node does not provide an id: ", node);
        return -1;
    }
}

std::string ModelManager::getName(const YAML::Node& node)
{
    if (!node[alica::Strings::name] || YAML::NodeType::Null == node[alica::Strings::name].Type()) {
        return alica::Strings::no_name;
    } else {
        return node[alica::Strings::name].as<std::string>();
    }
}

std::string ModelManager::getComment(const YAML::Node& node)
{
    if (!node[alica::Strings::comment] || YAML::NodeType::Null == node[alica::Strings::comment].Type()) {
        return alica::Strings::no_comment;
    } else {
        return node[alica::Strings::comment].as<std::string>();
    }
}

void ModelManager::storeElement(AlicaElement* ael, const std::string& type)
{
    // insert into general element map
    if (this->elements.find(ael->getId()) != this->elements.end()) {
        std::stringstream ss;
        ss << "MM: ERROR: ID utilised twice: " << ael->getId() << std::endl;
        ss << "ELEMENT >" << ael->getName() << "< >" << this->elements[ael->getId()]->getName() << "<" << std::endl;
        AlicaEngine::abort(ss.str());
    }
    this->elements.insert(std::pair<int64_t, AlicaElement*>(ael->getId(), ael));

    // insert into plan repository
    if (alica::Strings::plan.compare(type) == 0) {
        this->planRepository->_plans.emplace(ael->getId(), (Plan*) ael);
    } else if (alica::Strings::entryPoint.compare(type) == 0) {
        this->planRepository->_entryPoints.emplace(ael->getId(), (EntryPoint*) ael);
    } else {
        AlicaEngine::abort("MM: Element type unhandled for storing!");
    }
}

int64_t ModelManager::getReferencedId(const YAML::Node& node)
{
    std::string idString = node.as<std::string>();
    std::size_t idxOfHashtag = idString.find_last_of("#");
    if (idxOfHashtag == std::string::npos) {
        return node.as<int64_t>();
    } else {
        return stol(idString.substr(idxOfHashtag+1, idString.size()-idxOfHashtag));
    }
}

bool ModelManager::idExists(const int64_t id)
{
    return this->elements.find(id) != this->elements.end();
}

const AlicaElement* ModelManager::getElement(const int64_t id)
{
    auto mapEntry = this->elements.find(id);
    if (mapEntry != this->elements.end()) {
        return mapEntry->second;
    }
    return nullptr;
}

} // namespace alica