#include "engine/modelmanagement/ModelManager.h"

#include "engine/AlicaEngine.h"
#include "engine/ConfigChangeListener.h"
#include "engine/PlanRepository.h"
#include "engine/model/Behaviour.h"
#include "engine/model/Configuration.h"
#include "engine/model/Plan.h"
#include "engine/model/PlanType.h"
#include "engine/model/Quantifier.h"
#include "engine/model/RoleSet.h"
#include "engine/model/TaskRepository.h"
#include "engine/model/TransitionConditionRepository.h"
#include "engine/model/Variable.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/BehaviourFactory.h"
#include "engine/modelmanagement/factories/ConfigurationFactory.h"
#include "engine/modelmanagement/factories/PlanFactory.h"
#include "engine/modelmanagement/factories/PlanTypeFactory.h"
#include "engine/modelmanagement/factories/RoleSetFactory.h"
#include "engine/modelmanagement/factories/TaskRepositoryFactory.h"
#include "engine/modelmanagement/factories/TransitionConditionRepositoryFactory.h"
#include "engine/util/HashFunctions.h"

#include <alica_common_config/debug_output.h>
#include <essentials/FileSystem.h>
#include <functional>

namespace alica
{

ModelManager::ModelManager(PlanRepository& planRepository, AlicaEngine* ae, const std::string& domainConfigFolder)
        : _planRepository(planRepository)
        , _configChangeListener(ae->getConfigChangeListener()) // tmp only for compilation
        , domainConfigFolder(domainConfigFolder)
{
    auto reloadFunctionPtr = std::bind(&ModelManager::reload, this, std::placeholders::_1);
    _configChangeListener.subscribe(reloadFunctionPtr);
    reload(_configChangeListener.getConfig());
    Factory::setModelManager(this);
}

ModelManager::ModelManager(ConfigChangeListener& configChangeListener, const std::string& domainConfigFolder, PlanRepository& planRepository)
        : _configChangeListener(configChangeListener)
        , domainConfigFolder(domainConfigFolder)
        , _planRepository(planRepository)
{
    auto reloadFunctionPtr = std::bind(&ModelManager::reload, this, std::placeholders::_1);
    _configChangeListener.subscribe(reloadFunctionPtr);
    reload(_configChangeListener.getConfig());
    Factory::setModelManager(this);
}

void ModelManager::reload(const YAML::Node& config)
{
    basePlanPath = getBasePath("PlanDir");
    baseRolePath = getBasePath("RoleDir");
    baseTaskPath = getBasePath("TaskDir");
}

std::string ModelManager::getBasePath(const std::string& configKey)
{
    YAML::Node& config = _configChangeListener.getConfig();
    std::string basePath;
    try {
        basePath = config["Alica"][configKey].as<std::string>();
    } catch (const std::runtime_error& error) {
        AlicaEngine::abort("MM: Directory for config key '" + configKey + "' does not exist in the Alica config file.\n", error.what());
    }

    if (!essentials::FileSystem::endsWith(basePath, essentials::FileSystem::PATH_SEPARATOR)) {
        basePath = basePath + essentials::FileSystem::PATH_SEPARATOR;
    }

    if (!essentials::FileSystem::isPathRooted(basePath)) {
        basePath = essentials::FileSystem::combinePaths(domainConfigFolder, basePath);
    }

    if (!essentials::FileSystem::pathExists(basePath)) {
        AlicaEngine::abort("MM: Base path '" + basePath + "' does not exist for '" + configKey + "'");
    }

    ALICA_INFO_MSG("MM: Config key '" + configKey + "' maps to '" + basePath + "'");
    return basePath;
}

Plan* ModelManager::loadPlanTree(const std::string& masterPlanName)
{
    std::string masterPlanPath;
    if (!essentials::FileSystem::findFile(basePlanPath, masterPlanName + alica::Strings::plan_extension, masterPlanPath)) {
        AlicaEngine::abort("MM: Cannot find MasterPlan '" + masterPlanName + "'");
    }

    filesParsed.push_back(masterPlanPath);
    Plan* masterPlan = (Plan*) parseFile(masterPlanPath, alica::Strings::plan);
    while (filesToParse.size() > 0) {
        std::string fileToParse = filesToParse.front();
        filesToParse.pop_front();

        ALICA_DEBUG_MSG("MM: fileToParse: " << fileToParse);

        if (!essentials::FileSystem::pathExists(fileToParse)) {
            AlicaEngine::abort("MM: Cannot find referenced file:", fileToParse);
        }
        filesParsed.push_back(fileToParse);
        if (essentials::FileSystem::endsWith(fileToParse, alica::Strings::plan_extension)) {
            parseFile(fileToParse, alica::Strings::plan);
        } else if (essentials::FileSystem::endsWith(fileToParse, alica::Strings::taskrepository_extension)) {
            parseFile(fileToParse, alica::Strings::taskrepository);
        } else if (essentials::FileSystem::endsWith(fileToParse, alica::Strings::behaviour_extension)) {
            parseFile(fileToParse, alica::Strings::behaviour);
        } else if (essentials::FileSystem::endsWith(fileToParse, alica::Strings::configuration_extension)) {
            parseFile(fileToParse, alica::Strings::configuration);
        } else if (essentials::FileSystem::endsWith(fileToParse, alica::Strings::plantype_extension)) {
            parseFile(fileToParse, alica::Strings::plantype);
        } else if (essentials::FileSystem::endsWith(fileToParse, alica::Strings::condition_extension)) {
            parseFile(fileToParse, alica::Strings::transitionCondition);
        } else {
            AlicaEngine::abort("MM: Cannot parse file type: ", fileToParse);
        }
    }

    attachReferences();
    generateTemplateVariables();
    computeReachabilities();

    for (const Behaviour* beh : _planRepository.getBehaviours()) {
        ALICA_INFO_MSG("MM: " << beh->toString());
    }

    return masterPlan;
}

RoleSet* ModelManager::loadRoleSet(const std::string& roleSetName)
{
    std::string roleSetPath;
    if (!essentials::FileSystem::findFile(baseRolePath, roleSetName + alica::Strings::roleset_extension, roleSetPath)) {
        roleSetPath = findDefaultRoleSet(baseRolePath);
    }

    if (!essentials::FileSystem::pathExists(roleSetPath)) {
        AlicaEngine::abort("MM: Cannot find RoleSet '" + roleSetPath + "'");
    }

    RoleSet* roleSet = (RoleSet*) parseFile(roleSetPath, alica::Strings::roleset);
    RoleSetFactory::attachReferences();
    ALICA_INFO_MSG("MM: Parsed the following role set: \n" << roleSet->toString());
    return roleSet;
}

/**
 * Searches for default role sets in the given directory.
 * @param dir directory to search in (not recursively)
 * @return The first default role set it finds.
 */
std::string ModelManager::findDefaultRoleSet(const std::string& dir)
{
    std::string rolesetDir = dir;
    if (!essentials::FileSystem::isPathRooted(rolesetDir)) {
        rolesetDir = essentials::FileSystem::combinePaths(baseRolePath, rolesetDir);
    }
    if (!essentials::FileSystem::isDirectory(rolesetDir)) {
        AlicaEngine::abort("MM: RoleSet directory does not exist: " + rolesetDir);
    }

    std::vector<std::string> files = essentials::FileSystem::findAllFiles(rolesetDir, alica::Strings::roleset_extension);

    // find default role set and return first you find
    for (std::string file : files) {
        YAML::Node node;
        try {
            node = YAML::LoadFile(file);
            if (Factory::isValid(node[alica::Strings::defaultRoleSet]) && Factory::getValue<bool>(node, alica::Strings::defaultRoleSet)) {
                return file;
            }
        } catch (const YAML::BadFile& badFile) {
            AlicaEngine::abort("MM: Could not parse roleset file: ", badFile.msg);
        }
    }

    AlicaEngine::abort("MM: Could not find any default role set in '" + rolesetDir + "'");

    // need to return something, but it should never be reached (either found something, or abort)
    return files[0];
}

AlicaElement* ModelManager::parseFile(const std::string& currentFile, const std::string& type)
{
    YAML::Node node;
    try {
        node = YAML::LoadFile(currentFile);
    } catch (YAML::BadFile& badFile) {
        AlicaEngine::abort("MM: Could not parse file: ", badFile.msg);
    }
    if (alica::Strings::plan.compare(type) == 0) {
        Plan* plan = PlanFactory::create(_configChangeListener, node);
        plan->setFileName(currentFile);
        return plan;
    } else if (alica::Strings::behaviour.compare(type) == 0) {
        Behaviour* behaviour = BehaviourFactory::create(node);
        behaviour->setFileName(currentFile);
        return behaviour;
    } else if (alica::Strings::configuration.compare(type) == 0) {
        Configuration* configuration = ConfigurationFactory::create(node);
        configuration->setFileName(currentFile);
        return configuration;
    } else if (alica::Strings::plantype.compare(type) == 0) {
        PlanType* planType = PlanTypeFactory::create(node);
        planType->setFileName(currentFile);
        return planType;
    } else if (alica::Strings::taskrepository.compare(type) == 0) {
        TaskRepository* taskrepository = TaskRepositoryFactory::create(node);
        taskrepository->setFileName(currentFile);
        return taskrepository;
    } else if (alica::Strings::roleset.compare(type) == 0) {
        RoleSet* roleSet = RoleSetFactory::create(node);
        roleSet->setFileName(currentFile);
        return roleSet;
    } else if (alica::Strings::transitionCondition.compare(type) == 0) {
        TransitionConditionRepository* conditionRepository = TransitionConditionRepositoryFactory::create(node);
        conditionRepository->setFileName(currentFile);
        return conditionRepository;
    } else {
        AlicaEngine::abort("MM: Parsing type not handled: ", type);
        return nullptr;
    }
}

bool ModelManager::idExists(const int64_t id) const
{
    return elements.find(id) != elements.end();
}

const AlicaElement* ModelManager::getElement(const int64_t id)
{
    auto mapEntry = elements.find(id);
    if (mapEntry != elements.end()) {
        return mapEntry->second;
    }
    return nullptr;
}

void ModelManager::attachReferences()
{
    // these methods call recursively other factories for the job
    PlanFactory::attachReferences();
    PlanTypeFactory::attachReferences();
    BehaviourFactory::attachReferences();
}

void ModelManager::generateTemplateVariables()
{
    // generates template variables for all quantifiers
    for (std::pair<const int64_t, Quantifier*> p : _planRepository._quantifiers) {
        Quantifier* q = p.second;
        for (const std::string& s : q->getDomainIdentifiers()) {
            int64_t id = Hash64(s.c_str(), s.size());
            Variable* v;
            PlanRepository::MapType<Variable>::iterator vit = _planRepository._variables.find(id);
            if (vit != _planRepository._variables.end()) {
                v = vit->second;
            } else {
                v = new Variable(id, s, "Template");
                _planRepository._variables.emplace(id, v);
            }
            q->_templateVars.push_back(v);
        }
    }
}

/**
 * Computes the sets of reachable states for all entrypoints created.
 * This speeds up some calculations during run-time.
 */
void ModelManager::computeReachabilities()
{
    for (const std::pair<const int64_t, EntryPoint*>& ep : _planRepository._entryPoints) {
        ep.second->computeReachabilitySet();
        // set backpointers:
        for (const State* s : ep.second->_reachableStates) {
            _planRepository._states[s->getId()]->_entryPoint = ep.second;
        }
    }
}

} // namespace alica
