#include "engine/parser/PlanParser.h"
#include "engine/AlicaEngine.h"
#include "engine/model/Plan.h"
#include "engine/parser/ModelFactory.h"

#include <alica_common_config/debug_output.h>
#include <engine/containers/AlicaEngineInfo.h>

#include <yaml-cpp/yaml.h>

namespace alica
{

    using std::shared_ptr;
    using std::string;

/**
 * Constructor
 * @param A PlanRepository, in which parsed elements are stored.
 */
    PlanParser::PlanParser(PlanRepository* rep)
    {
        using namespace supplementary;

        this->masterPlan = nullptr;
        this->rep = rep;
        this->mf = std::make_shared<ModelFactory>(this, rep);
        this->sc = SystemConfig::getInstance();
        this->domainConfigFolder = this->sc->getConfigPath();

        try {
            this->planDir = (*this->sc)["Alica"]->get<string>("Alica.PlanDir", NULL);
        } catch (const std::runtime_error& error) {
            AlicaEngine::abort("PP: Plan Directory does not exist.\n", error.what());
        }

        try {
            this->roleDir = (*this->sc)["Alica"]->get<string>("Alica.RoleDir", NULL);
        } catch (const std::runtime_error& error) {
            AlicaEngine::abort("PP: Role Directory does not exist.\n", error.what());
        }

        try {
            this->taskDir = (*this->sc)["Alica"]->get<string>("Alica.TaskDir", NULL);
        } catch (const std::runtime_error& error) {
            AlicaEngine::abort("PP: Task Directory does not exist.\n", error.what());
        }

        if (!FileSystem::endsWith(domainConfigFolder, FileSystem::PATH_SEPARATOR)) {
            domainConfigFolder = domainConfigFolder + FileSystem::PATH_SEPARATOR;
        }
        if (!FileSystem::endsWith(planDir, FileSystem::PATH_SEPARATOR)) {
            planDir = planDir + FileSystem::PATH_SEPARATOR;
        }
        if (!FileSystem::endsWith(roleDir, FileSystem::PATH_SEPARATOR)) {
            roleDir = roleDir + FileSystem::PATH_SEPARATOR;
        }
        if (!(supplementary::FileSystem::isPathRooted(this->planDir))) {
            basePlanPath = domainConfigFolder + planDir;
        } else {
            basePlanPath = planDir;
        }
        if (!(supplementary::FileSystem::isPathRooted(this->roleDir))) {
            baseRolePath = domainConfigFolder + roleDir;
        } else {
            baseRolePath = roleDir;
        }
        if (!(supplementary::FileSystem::isPathRooted(this->taskDir))) {
            baseTaskPath = domainConfigFolder + taskDir;
        } else {
            baseTaskPath = taskDir;
        }

        ALICA_DEBUG_MSG("PP: basePlanPath: " << basePlanPath);
        ALICA_DEBUG_MSG("PP: baseRolePath: " << baseRolePath);
        ALICA_DEBUG_MSG("PP: baseTaskPath: " << baseTaskPath);

        if (!(supplementary::FileSystem::pathExists(basePlanPath))) {
            AlicaEngine::abort("PP: BasePlanPath does not exist " + basePlanPath);
        }
        if (!(supplementary::FileSystem::pathExists(baseRolePath))) {
            AlicaEngine::abort("PP: BaseRolePath does not exist " + baseRolePath);
        }
        if (!(supplementary::FileSystem::pathExists(baseTaskPath))) {
            AlicaEngine::abort("PP: BaseTaskPath does not exist " + baseTaskPath);
        }
    }

    PlanParser::~PlanParser() {}

/**
 * Parse a roleset
 * @param roleSetName The name of the roleset to parse. May be empty, in which case a default roleset is looked for.
 * @param roleSetDir The relative directory in which to search for a roleset.
 * @return The parsed roleSet
 */
    const RoleSet* PlanParser::parseRoleSet(std::string roleSetName)
    {
        using namespace supplementary;
        std::string roleSetLocation;
        if (roleSetName.empty()) {
            roleSetLocation = findDefaultRoleSet(baseRolePath);
        } else {
            if (!supplementary::FileSystem::endsWith(roleSetName, ".rset")) {
                roleSetName = roleSetName + ".rset";
            }

            supplementary::FileSystem::findFile(this->baseRolePath, roleSetName, roleSetLocation);
        }

        if (!supplementary::FileSystem::pathExists(roleSetLocation)) {
            AlicaEngine::abort("PP: Cannot find roleset: " + roleSetLocation);
        }

        ALICA_DEBUG_MSG("PP: Parsing RoleSet " << roleSetLocation);

        this->currentDirectory = supplementary::FileSystem::getParent(roleSetLocation);

        tinyxml2::XMLDocument doc;
        doc.LoadFile(roleSetLocation.c_str());
        if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
            AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
        }

        RoleSet* r = this->mf->createRoleSet(&doc, this->masterPlan);

        filesParsed.push_back(roleSetLocation);

        while (!this->filesToParse.empty()) {
            string fileToParse = this->filesToParse.front();
            this->filesToParse.pop_front();
            this->currentDirectory = supplementary::FileSystem::getParent(fileToParse);
            this->currentFile = fileToParse;

            if (!supplementary::FileSystem::pathExists(fileToParse)) {
                AlicaEngine::abort("PP: Cannot Find referenced file: '" + fileToParse + "'");
            }
            if (supplementary::FileSystem::endsWith(fileToParse, ".rdefset")) {
                parseRoleDefFile(fileToParse);
            } else if (supplementary::FileSystem::endsWith(fileToParse, ".cdefset")) {
                parseCapabilityDefFile(fileToParse);
            } else {
                AlicaEngine::abort("PP: Cannot Parse file " + fileToParse);
            }
            filesParsed.push_back(fileToParse);
        }

        this->mf->attachRoleReferences();
        this->mf->attachCharacteristicReferences();
        return r;
    }

    void PlanParser::parseRoleDefFile(const std::string& currentFile)
    {
        ALICA_DEBUG_MSG("PP: parsing RoleDef file: " << currentFile);

        tinyxml2::XMLDocument doc;
        doc.LoadFile(currentFile.c_str());
        if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
            AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
        }
        this->mf->createRoleDefinitionSet(&doc);
    }

    void PlanParser::parseCapabilityDefFile(const std::string& currentFile)
    {
        ALICA_DEBUG_MSG("PP: parsing RoleDef file: " << currentFile);
        tinyxml2::XMLDocument doc;
        doc.LoadFile(currentFile.c_str());
        if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
            AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
        }
        this->mf->createCapabilityDefinitionSet(&doc);
    }

    std::string PlanParser::findDefaultRoleSet(std::string dir)
    {
        if (!supplementary::FileSystem::isPathRooted(dir)) {
            dir = supplementary::FileSystem::combinePaths(this->baseRolePath, dir);
        }
        if (!supplementary::FileSystem::isDirectory(dir)) {
            AlicaEngine::abort("PP: RoleSet directory does not exist: " + dir);
        }

        std::vector<std::string> files = supplementary::FileSystem::findAllFiles(dir, ".rset");

        for (string s : files) {
            tinyxml2::XMLDocument doc;
            doc.LoadFile(s.c_str());
            if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
                AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
            }

            tinyxml2::XMLElement* element = doc.FirstChildElement();
            const char* attr = element->Attribute("default");
            if (attr) {
                string attrString = attr;
                if (attrString == "true") {
                    return s;
                }
            }
        }

        if (files.size() != 1) {
            AlicaEngine::abort("PP: Cannot find a default roleset in directory: " + dir);
        }

        return files[0];
    }

    const Plan* PlanParser::getHackedPlan()
    {
        return this->mf->getHackedPlan();
    }

    const RoleSet* PlanParser::getHackedRoleSet() {
        return this->mf->getHackedRoleSet();
    }

/**
 * Parses a plan tree
 * @param masterplan The name of the top-level plan
 * @return The top-level plan
 */
    const Plan* PlanParser::parsePlanTree(const std::string& masterplan)
    {
        std::string masterPlanPath;
        bool found = supplementary::FileSystem::findFile(this->basePlanPath, masterplan + ".pml", masterPlanPath);

        if (!found) {
            AlicaEngine::abort("PP: Cannot find MasterPlan '" + masterplan + "'");
        }
        this->currentFile = masterPlanPath;
        this->currentDirectory = supplementary::FileSystem::getParent(masterPlanPath);

        this->filesParsed.push_back(masterPlanPath);
//        parseTaskFile("/home/witali/rosws/src/cnc-turtlebots/etc/Misc/TurtleBot2.tsk");
//        parsePlanFile("/home/witali/rosws/src/cnc-turtlebots/etc/plans/TestMasterPlan.pml");
//        parsePlanTypeFile("/home/witali/rosws/src/cnc-turtlebots/etc/plans/Plantypes/TestPlanType.pty");
//        parseBehaviourFile("/home/witali/rosws/src/cnc-turtlebots/etc/plans/Behaviours/RunForest2.beh");
        this->masterPlan = parsePlanFile(masterPlanPath);
        parseFileLoop();

        this->mf->computeReachabilities();
        return this->masterPlan;
    }

    void PlanParser::parseFileLoop()
    {
        while (this->filesToParse.size() > 0) {
            string fileToParse = this->filesToParse.front();
            this->filesToParse.pop_front();
            this->currentDirectory = supplementary::FileSystem::getParent(fileToParse);
            this->currentFile = fileToParse;

            if (!supplementary::FileSystem::pathExists(fileToParse)) {
                AlicaEngine::abort("PP: Cannot Find referenced file ", fileToParse);
            }
            filesParsed.push_back(fileToParse);
            if (supplementary::FileSystem::endsWith(fileToParse, ".pml")) {
                parsePlanFile(fileToParse);
            } else if (supplementary::FileSystem::endsWith(fileToParse, ".tsk")) {
                parseTaskFile(fileToParse);
            } else if (supplementary::FileSystem::endsWith(fileToParse, ".beh")) {
                parseBehaviourFile(fileToParse);
            } else if (supplementary::FileSystem::endsWith(fileToParse, ".pty")) {
                parsePlanTypeFile(fileToParse);
            } else {
                AlicaEngine::abort("PP: Cannot Parse file", fileToParse);
            }
        }
        this->mf->attachPlanReferences();
    }

    void PlanParser::parsePlanTypeFile(const std::string& currentFile)
    {
        YAML::Node doc;
        try {
            doc = YAML::LoadFile(currentFile);
        } catch (YAML::BadFile badFile) {
            AlicaEngine::abort("PP: parsePlanTypeFile doc.ErrorCode: ", badFile.msg);
        }

        this->mf->createPlanType(doc);
    }

    void PlanParser::parseBehaviourFile(const std::string& currentFile)
    {
        YAML::Node doc;
        try {
            doc = YAML::LoadFile(currentFile);
        } catch (YAML::BadFile badFile) {
            AlicaEngine::abort("PP: parseBehaviourFile doc.ErrorCode: ", badFile.msg);
        }

        this->mf->createBehaviour(doc);
    }

    void PlanParser::parseTaskFile(const std::string& currentFile)
    {
        YAML::Node doc;
        try {
            doc = YAML::LoadFile(currentFile);
        } catch (YAML::BadFile badFile) {
            AlicaEngine::abort("PP: parseTaskFile doc.ErrorCode: ", badFile.msg);
        }

        this->mf->createTasks(doc);
    }

    Plan* PlanParser::parsePlanFile(const std::string& planFile)
    {
        YAML::Node doc;
        try {
            doc = YAML::LoadFile(planFile);
        } catch (YAML::BadFile badFile) {
            AlicaEngine::abort("PP: parsePlanFile doc.ErrorCode: ", badFile.msg);
        }

        Plan* p = nullptr;
        try {
            p = this->mf->createPlan(doc);
        } catch (std::exception& e) {
            AlicaEngine::abort("PP: cannot create plan for " + planFile + "\nException: " + e.what());
        }
        return p;
    }

/**
 * Provides access to all parsed elements, indexed by their id.
 * @return A shared_ptr<map<long, alica::AlicaElement>
 */
    std::map<int64_t, alica::AlicaElement*>* PlanParser::getParsedElements()
    {
        return mf->getElements();
    }

    void PlanParser::ignoreMasterPlanId(bool val)
    {
        this->mf->setIgnoreMasterPlanId(val);
    }

    void PlanParser::setCurrentFile(const std::string& currentFile)
    {
        if (currentFile.compare(0, basePlanPath.size(), basePlanPath)) {
            this->currentFile = currentFile.substr(basePlanPath.size());
        } else if (currentFile.compare(0, baseRolePath.size(), baseRolePath)) {
            this->currentFile = currentFile.substr(baseRolePath.size());
        } else {
            this->currentFile = currentFile;
        }
    }

/**
 * parse id of the plan
 * @param node the given xml node
 * @return id: returns the id from the plan
 */
    int64_t PlanParser::parserId(YAML::Node node)
    {
        try {
            if(node["id"]) {
                return node["id"].as<std::int64_t >();
            }
        } catch (std::exception& e) {
            AlicaEngine::abort("PP: Cannot convert ID to int64_t: " +  node["id"].as<std::string >() + "\nException: " + e.what());
        }

        std::cerr << "Cannot resolve remote reference!\nAttributes of node in question are:" << std::endl;
        std::string nodesName = "Nodes:";
        for (auto it = node.begin(); it != node.end(); ++it) {
            auto key = it->first;
            auto value = it->second;
            nodesName = nodesName + " " + key.as<std::string>();
        }
        AlicaEngine::abort("PP: Couldn't resolve remote reference: " + nodesName);
        return -1;
    }

/**
 * Helper
 * @param idString is a String that have to be converted in a int64_t
 * @param id the id
 */
    int64_t PlanParser::fetchId(const string& idString)
    {
        int hashPos = idString.find_first_of("#");

        string locator = idString.substr(0, hashPos);
        if (!locator.empty()) {
            if (!supplementary::FileSystem::endsWith(this->currentDirectory, "/")) {
                this->currentDirectory = this->currentDirectory + "/";
            }
            string path;
            if (supplementary::FileSystem::endsWith(locator, ".pml") || supplementary::FileSystem::endsWith(locator, ".beh")) {
                path = supplementary::FileSystem::combinePaths(this->basePlanPath, locator);
            } else if (supplementary::FileSystem::endsWith(locator, ".tsk")) {
                path = supplementary::FileSystem::combinePaths(this->baseTaskPath, locator);
            } else if (supplementary::FileSystem::endsWith(locator, ".rdefset") || supplementary::FileSystem::endsWith(locator, ".cdefset")) {
                path = supplementary::FileSystem::combinePaths(this->baseRolePath, locator);
            } else {
                std::cout << "PP: Unknown locator ending: " << locator << std::endl;
            }

            if (std::find(std::begin(filesParsed), std::end(filesParsed), path) == std::end(filesParsed) &&
                std::find(std::begin(filesToParse), std::end(filesToParse), path) == std::end(filesToParse)) {
                filesToParse.push_back(path);
            }
        }
        int64_t id;
        std::string tokenId = idString.substr(hashPos + 1, idString.length() - hashPos);
        try {
            id = stoll(tokenId);
        } catch (std::exception& e) {
            AlicaEngine::abort("PP: Cannot convert ID to int64_t: " + tokenId + "\nException: " + e.what());
        }
        return id;
    }
} // namespace alica
/* namespace Alica */
