#include "engine/modelmanagement/ModelManager.h"

#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/EntryPoint.h"
#include "engine/model/Plan.h"
#include "engine/modelmanagement/Strings.h"
#include "engine/modelmanagement/factories/Factory.h"
#include "engine/modelmanagement/factories/PlanFactory.h"

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
    Factory::setElementsMap(elements);
    Factory::setPlanRepository(this->planRepository);
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
        Plan* plan = PlanFactory::create(doc);
        plan->setFileName(currentFile);
        return plan;
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