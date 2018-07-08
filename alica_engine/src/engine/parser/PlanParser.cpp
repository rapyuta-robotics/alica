#include "engine/parser/PlanParser.h"
#include "engine/AlicaEngine.h"
#include "engine/model/Plan.h"
#include "engine/parser/ModelFactory.h"

#include <alica_common_config/debug_output.h>

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
    this->mf = shared_ptr<ModelFactory>(new ModelFactory(this, rep));
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

    if (domainConfigFolder.find_last_of(FileSystem::PATH_SEPARATOR) != domainConfigFolder.length() - 1) {
        domainConfigFolder = domainConfigFolder + FileSystem::PATH_SEPARATOR;
    }
    if (planDir.find_last_of(FileSystem::PATH_SEPARATOR) != planDir.length() - 1) {
        planDir = planDir + FileSystem::PATH_SEPARATOR;
    }
    if (roleDir.find_last_of(FileSystem::PATH_SEPARATOR) != roleDir.length() - 1) {
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

    ALICA_DEBUG_MSG("PP: basePlanPath: " << basePlanPath);
    ALICA_DEBUG_MSG("PP: baseRolePath: " << baseRolePath);
    ;

    if (!(supplementary::FileSystem::pathExists(basePlanPath))) {
        AlicaEngine::abort("PP: BasePlanPath does not exist " + basePlanPath);
    }
    if (!(supplementary::FileSystem::pathExists(baseRolePath))) {
        AlicaEngine::abort("PP: BaseRolePath does not exist " + baseRolePath);
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

    while (this->filesToParse.size() > 0) {
        string fileToParse = this->filesToParse.front();
        this->filesToParse.pop_front();
        this->currentDirectory = supplementary::FileSystem::getParent(fileToParse);
        this->currentFile = fileToParse;

        if (!supplementary::FileSystem::pathExists(fileToParse)) {
            AlicaEngine::abort("PP: Cannot Find referenced file ", fileToParse);
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
            if (attrString.compare("true") == 0) {
                return s;
            }
        }
    }
    if (files.size() == 1) {
        return files[0];
    }
    AlicaEngine::abort("PP: Cannot find a default roleset in directory: " + dir);
    return "";
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
#ifdef PP_DEBUG
    std::cout << "PP: masterPlanPath: " << masterPlanPath << std::endl;
#endif
    if (!found) {
        AlicaEngine::abort("PP: Cannot find MasterPlan '" + masterplan + "'");
    }
    this->currentFile = masterPlanPath;
    this->currentDirectory = supplementary::FileSystem::getParent(masterPlanPath);
#ifdef PP_DEBUG
    std::cout << "PP: CurFile: " << this->currentFile << " CurDir: " << this->currentDirectory << std::endl;
#endif

    this->masterPlan = parsePlanFile(masterPlanPath);
    this->filesParsed.push_back(masterPlanPath);
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
        if (supplementary::FileSystem::endsWith(fileToParse, ".pml")) {
            parsePlanFile(fileToParse);
        } else if (supplementary::FileSystem::endsWith(fileToParse, ".tsk")) {
            parseTaskFile(fileToParse);
        } else if (supplementary::FileSystem::endsWith(fileToParse, ".beh")) {
            parseBehaviourFile(fileToParse);
        } else if (supplementary::FileSystem::endsWith(fileToParse, ".pty")) {
            parsePlanTypeFile(fileToParse);
        } else if (supplementary::FileSystem::endsWith(fileToParse, ".pp")) {
            parsePlanningProblem(fileToParse);
        } else {
            AlicaEngine::abort("PP: Cannot Parse file", fileToParse);
        }
        filesParsed.push_back(fileToParse);
    }
    this->mf->attachPlanReferences();
}

/**
 * Parses a planning problem
 * @param planName The name of the planning problem
 * @return The top-level PlanningProblem
 */
void PlanParser::parsePlanningProblem(const std::string& currentFile)
{
#ifdef PP_DEBUG
    std::cout << "PP: parsing Planning Problem file: " << currentFile << std::endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
    }
    this->mf->createPlanningProblem(&doc);
}

void PlanParser::parsePlanTypeFile(const std::string& currentFile)
{
#ifdef PP_DEBUG
    std::cout << "PP: parsing PlanType file: " << currentFile << std::endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
    }
    this->mf->createPlanType(&doc);
}
void PlanParser::parseBehaviourFile(const std::string& currentFile)
{
#ifdef PP_DEBUG
    std::cout << "PP: parsing Behaviour file: " << currentFile << std::endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
    }
    this->mf->createBehaviour(&doc);
}

void PlanParser::parseTaskFile(const std::string& currentFile)
{
#ifdef PP_DEBUG
    std::cout << "PP: parsing Task file: " << currentFile << std::endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
#ifdef PP_DEBUG
    std::cout << "TASKREPO " << currentFile << std::endl;
#endif
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
    }
    this->mf->createTasks(&doc);
}

Plan* PlanParser::parsePlanFile(const std::string& planFile)
{
#ifdef PP_DEBUG
    std::cout << "PP: parsing Plan file: " << planFile << std::endl;
#endif

    tinyxml2::XMLDocument doc;
    doc.LoadFile(planFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
    }
    Plan* p = nullptr;
    try {
        p = this->mf->createPlan(&doc);
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
int64_t PlanParser::parserId(tinyxml2::XMLElement* node)
{
    int64_t id = -1;
    string idString = "";
    const char* idChar = node->Attribute("id");
    if (idChar)
        idString = idChar;
    if (idString.compare("") != 0) {
        try {
            id = stol(idString);
        } catch (std::exception& e) {
            AlicaEngine::abort("PP: Cannot convert ID to long: " + idString + "\nException: " + e.what());
        }
        return id;
    } else {
        string idString = "";
        const char* idChar = node->Attribute("href");
        if (idChar)
            idString = idChar;
        if (idString.compare("") != 0) {
            id = fetchId(idString, id);
            return id;
        } else {
            const tinyxml2::XMLNode* currNode = node->FirstChild();
            while (currNode) {
                const tinyxml2::XMLText* textNode = currNode->ToText();
                if (textNode) {
                    id = fetchId(textNode->Value(), id);
                    return id;
                }

                currNode = currNode->NextSibling();
            }
        }
    }

    std::cerr << "Cannot resolve remote reference!\nAttributes of node in question are:" << std::endl;
    const tinyxml2::XMLAttribute* curAttribute = node->FirstAttribute();
    while (curAttribute) {
        std::cout << curAttribute->Name() << " : " << curAttribute->Value() << std::endl;
        curAttribute = curAttribute->Next();
    }
    AlicaEngine::abort("PP: Couldn't resolve remote reference: " + string(node->Name()));
    return -1;
}

/**
 * Helper
 * @param idString is a String that have to be converted in a int64_t
 * @param id the id
 */
int64_t PlanParser::fetchId(const string& idString, int64_t id)
{
    int hashPos = idString.find_first_of("#");
    char* temp = nullptr;
    char* temp2 = nullptr;
    string locator = idString.substr(0, hashPos);
    if (!locator.empty()) {
        if (!supplementary::FileSystem::endsWith(this->currentDirectory, "/")) {
            this->currentDirectory = this->currentDirectory + "/";
        }
        string path = this->currentDirectory + locator;
        // not working no clue why
        // char s[2048];
        // char s2[2048];
        temp = realpath(path.c_str(), nullptr);
        std::string pathNew;
        if (temp) {
            pathNew = temp;
            free(temp);
        }
        // This is not very efficient but necessary to keep the paths as they are
        // Here we have to check whether the file has already been parsed / is in the list for toparse files
        // problem is the normalization /home/etc/plans != /home/etc/Misc/../plans
        // list<string>::iterator findIterParsed = find(filesParsed.begin(), filesParsed.end(), pathNew);
        bool found = false;
        for (auto& it : filesParsed) {
            temp2 = realpath(it.c_str(), nullptr);
            std::string pathNew2;
            if (temp2) {
                pathNew2 = temp2;
                free(temp2);
            }

            if (pathNew2 == pathNew) {
                found = true;
                break;
            }
        }

        // list<string>::iterator findIterToParse = find(filesToParse.begin(), filesToParse.end(), pathNew);
        if (!found) {
            for (auto& it : filesToParse) {
                temp2 = realpath(it.c_str(), nullptr);
                if (temp2 != nullptr) {
                    std::string pathNew2 = temp2;
                    free(temp2);
                    if (pathNew2 == pathNew) {
                        found = true;
                        break;
                    }
                }
            }
        }

        if (!found) {
#ifdef PP_DEBUG
            std::cout << "PP: Adding " + path + " to parse queue " << std::endl;
#endif
            filesToParse.push_back(path);
        }
    }
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
