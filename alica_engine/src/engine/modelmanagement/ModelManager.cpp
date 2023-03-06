#include "engine/modelmanagement/ModelManager.h"

#include "engine/AlicaEngine.h"
#include "engine/ConfigChangeListener.h"
#include "engine/PlanRepository.h"
#include "engine/logging/Logging.h"
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
#include "engine/modelmanagement/factories/VariableFactory.h"
#include "engine/util/HashFunctions.h"

#include <engine/FileSystem.h>
#include <functional>

namespace alica
{

ModelManager::ModelManager(ConfigChangeListener& configChangeListener, const std::vector<std::string>& domainConfigFolders, PlanRepository& planRepository)
        : _configChangeListener(configChangeListener)
        , _domainConfigFolders(domainConfigFolders)
        , _planRepository(planRepository)
{
    Factory::setModelManager(this);
}

Plan* ModelManager::loadPlanTree(const std::string& masterPlanName)
{
    std::string masterPlanPath;
    if (masterPlanPath = findFile(masterPlanName + alica::Strings::plan_extension); masterPlanPath.empty()) {
        AlicaEngine::abort(LOGNAME, "Cannot find MasterPlan '", masterPlanName, "'");
    }

    filesParsed.push_back(masterPlanName + alica::Strings::plan_extension);
    Plan* masterPlan = (Plan*) parseFile(masterPlanPath, alica::Strings::plan);
    while (filesToParse.size() > 0) {
        const auto fileToParse = findFile(filesToParse.front());
        if (fileToParse.empty()) {
            AlicaEngine::abort(LOGNAME, "Cannot find file '", filesToParse.front(), "'");
        }

        Logging::logDebug(LOGNAME) << "fileToParse: " << fileToParse;
        filesParsed.push_back(filesToParse.front());
        filesToParse.pop_front();

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
        } else if (essentials::FileSystem::endsWith(fileToParse, alica::Strings::roleset_extension)) {
            AlicaEngine::abort(LOGNAME, "Cannot have multiple rolesets");
        } else {
            AlicaEngine::abort(LOGNAME, "Cannot parse file type: ", fileToParse);
        }
    }

    attachReferences();
    generateTemplateVariables();
    computeReachabilities();

    return masterPlan;
}

RoleSet* ModelManager::loadRoleSet(const std::string& roleSetName)
{
    std::string roleSetPath = findFile(roleSetName + alica::Strings::roleset_extension);
    if (roleSetPath.empty()) {
        Logging::logWarn(LOGNAME) << "Roleset with name " << roleSetName << " not found, will look for any other roleset files in the given config directories";
        roleSetPath = findDefaultRoleSet();
    }

    if (roleSetPath.empty()) {
        AlicaEngine::abort(LOGNAME, "Cannot find any rolesets in the config directories");
    }

    RoleSet* roleSet = (RoleSet*) parseFile(roleSetPath, alica::Strings::roleset);
    RoleSetFactory::attachReferences();
    return roleSet;
}

std::string ModelManager::findDefaultRoleSet() const
{
    for (const auto& folder : _domainConfigFolders) {
        std::vector<std::string> files = essentials::FileSystem::findAllFiles(folder, alica::Strings::roleset_extension);

        // find default role set and return the first valid one found
        for (const auto& file : files) {
            try {
                auto node = YAML::LoadFile(file);
                if (Factory::isValid(node[alica::Strings::defaultRoleSet]) && Factory::getValue<bool>(node, alica::Strings::defaultRoleSet)) {
                    return file;
                } else {
                    Logging::logWarn(LOGNAME) << "Roleset found, however it is invalid, will continue with the search for another roleset";
                }
            } catch (const YAML::BadFile& badFile) {
                Logging::logWarn(LOGNAME) << "Roleset found, however parsing failed with exception: " << badFile.what()
                                          << ", will continue the search for another roleset";
            }
        }
    }

    return std::string{};
}

std::string ModelManager::findFile(const std::string& fileName) const
{
    std::string filePath;
    for (const auto& folder : _domainConfigFolders) {
        if (essentials::FileSystem::findFile(folder, fileName, filePath)) {
            return filePath;
        }
    }
    return std::string{};
}

AlicaElement* ModelManager::parseFile(const std::string& currentFile, const std::string& type)
{
    YAML::Node node;
    try {
        node = YAML::LoadFile(currentFile);
    } catch (YAML::BadFile& badFile) {
        AlicaEngine::abort(LOGNAME, "Could not parse file: ", badFile.msg);
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
        AlicaEngine::abort(LOGNAME, "Parsing type not handled: ", type);
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
        return mapEntry->second.get();
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
                v = VariableFactory::create(id, s, "Template");
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
