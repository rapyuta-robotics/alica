#include "engine/modelmanagement/ModelManager.h"

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/model/Plan.h"
#include "engine/model/Behaviour.h"
#include "engine/model/PlanType.h"
#include "engine/model/TaskRepository.h"
#include "engine/modelmanagement/factories/PlanFactory.h"
#include "engine/modelmanagement/factories/BehaviourFactory.h"
#include "engine/modelmanagement/factories/TaskRepositoryFactory.h"
#include "engine/modelmanagement/factories/PlanTypeFactory.h"

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
    Factory::setModelManager(this);
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

    this->filesParsed.push_back(masterPlanPath);
    Plan* masterPlan = (Plan*) parseFile(masterPlanPath, alica::Strings::plan);
    while (this->filesToParse.size() > 0) {
        std::string fileToParse = this->filesToParse.front();
        this->filesToParse.pop_front();

        std::cout << "MM: fileToParse: " << fileToParse << std::endl;

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
        } else if (essentials::FileSystem::endsWith(fileToParse, alica::Strings::plantype_extension)) {
            parseFile(fileToParse, alica::Strings::plantype);
        } else {
            AlicaEngine::abort("MM: Cannot parse file type: ", fileToParse);
        }
    }

    // TODO
    //    this->mf->attachPlanReferences();
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
    YAML::Node node;
    try {
        node = YAML::LoadFile(currentFile);
    } catch (YAML::BadFile badFile) {
        AlicaEngine::abort("MM: Could not parse file: ", badFile.msg);
    }

    if (alica::Strings::plan.compare(type) == 0) {
        Plan* plan = PlanFactory::create(node);
        plan->setFileName(currentFile);
        return plan;
    } else if (alica::Strings::behaviour.compare(type) == 0) {
        Behaviour* behaviour = BehaviourFactory::create(node);
        behaviour->setFileName(currentFile);
        return behaviour;
    } else if (alica::Strings::plantype.compare(type) == 0) {
        PlanType* planType = PlanTypeFactory::create(node);
        planType->setFileName(currentFile);
        return planType;
    } else if (alica::Strings::taskrepository.compare(type) == 0) {
        TaskRepository* taskrepository = TaskRepositoryFactory::create(node);
        taskrepository->setFileName(currentFile);
        return taskrepository;
    } else {
        AlicaEngine::abort("MM: Parsing type not handled: ", type);
        return nullptr;
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