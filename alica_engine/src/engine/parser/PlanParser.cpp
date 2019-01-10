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
    this->_masterPlan = nullptr;
    this->_rep = rep;
    this->_mf = shared_ptr<ModelFactory>(new ModelFactory(this, rep));
    essentials::SystemConfig& sc = essentials::SystemConfig::getInstance();
    std::string domainConfigFolder = sc.getConfigPath();

    try {
        this->_planDir = sc["Alica"]->get<string>("Alica.PlanDir", NULL);
    } catch (const std::runtime_error& error) {
        AlicaEngine::abort("PP: Plan Directory does not exist.\n", error.what());
    }

    try {
        this->_roleDir = sc["Alica"]->get<string>("Alica.RoleDir", NULL);
    } catch (const std::runtime_error& error) {
        AlicaEngine::abort("PP: Role Directory does not exist.\n", error.what());
    }

    if (domainConfigFolder.find_last_of(essentials::FileSystem::PATH_SEPARATOR) != domainConfigFolder.length() - 1) {
        domainConfigFolder = domainConfigFolder + essentials::FileSystem::PATH_SEPARATOR;
    }
    if (_planDir.find_last_of(essentials::FileSystem::PATH_SEPARATOR) != _planDir.length() - 1) {
        _planDir = _planDir + essentials::FileSystem::PATH_SEPARATOR;
    }
    if (_roleDir.find_last_of(essentials::FileSystem::PATH_SEPARATOR) != _roleDir.length() - 1) {
        _roleDir = _roleDir + essentials::FileSystem::PATH_SEPARATOR;
    }
    if (!(essentials::FileSystem::isPathRooted(this->_planDir))) {
        _basePlanPath = domainConfigFolder + _planDir;
    } else {
        _basePlanPath = _planDir;
    }
    if (!(essentials::FileSystem::isPathRooted(this->_roleDir))) {
        _baseRolePath = domainConfigFolder + _roleDir;
    } else {
        _baseRolePath = _roleDir;
    }

    ALICA_DEBUG_MSG("PP: basePlanPath: " << _basePlanPath);
    ALICA_DEBUG_MSG("PP: baseRolePath: " << _baseRolePath);
    ;

    if (!(essentials::FileSystem::pathExists(_basePlanPath))) {
        AlicaEngine::abort("PP: BasePlanPath does not exist " + _basePlanPath);
    }
    if (!(essentials::FileSystem::pathExists(_baseRolePath))) {
        AlicaEngine::abort("PP: BaseRolePath does not exist " + _baseRolePath);
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
    std::string roleSetLocation;
    if (roleSetName.empty()) {
        roleSetLocation = findDefaultRoleSet(_baseRolePath);
    } else {
        if (!essentials::FileSystem::endsWith(roleSetName, ".rset")) {
            roleSetName = roleSetName + ".rset";
        }

        essentials::FileSystem::findFile(this->_baseRolePath, roleSetName, roleSetLocation);
    }

    if (!essentials::FileSystem::pathExists(roleSetLocation)) {
        AlicaEngine::abort("PP: Cannot find roleset: " + roleSetLocation);
    }

    ALICA_DEBUG_MSG("PP: Parsing RoleSet " << roleSetLocation);

    this->_currentDirectory = essentials::FileSystem::getParent(roleSetLocation);

    tinyxml2::XMLDocument doc;
    doc.LoadFile(roleSetLocation.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
    }

    RoleSet* r = this->_mf->createRoleSet(&doc, this->_masterPlan);

    _filesParsed.push_back(roleSetLocation);

    while (this->_filesToParse.size() > 0) {
        string fileToParse = this->_filesToParse.front();
        this->_filesToParse.pop_front();
        this->_currentDirectory = essentials::FileSystem::getParent(fileToParse);
        this->_currentFile = fileToParse;

        if (!essentials::FileSystem::pathExists(fileToParse)) {
            AlicaEngine::abort("PP: Cannot Find referenced file ", fileToParse);
        }
        if (essentials::FileSystem::endsWith(fileToParse, ".rdefset")) {
            parseRoleDefFile(fileToParse);
        } else if (essentials::FileSystem::endsWith(fileToParse, ".cdefset")) {
            parseCapabilityDefFile(fileToParse);
        } else {
            AlicaEngine::abort("PP: Cannot Parse file " + fileToParse);
        }
        _filesParsed.push_back(fileToParse);
    }

    this->_mf->attachRoleReferences();
    this->_mf->attachCharacteristicReferences();
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
    this->_mf->createRoleDefinitionSet(&doc);
}
void PlanParser::parseCapabilityDefFile(const std::string& currentFile)
{
    ALICA_DEBUG_MSG("PP: parsing RoleDef file: " << currentFile);

    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        AlicaEngine::abort("PP: doc.ErrorCode: ", tinyxml2::XMLErrorStr[doc.ErrorID()]);
    }
    this->_mf->createCapabilityDefinitionSet(&doc);
}

std::string PlanParser::findDefaultRoleSet(std::string dir)
{
    if (!essentials::FileSystem::isPathRooted(dir)) {
        dir = essentials::FileSystem::combinePaths(this->_baseRolePath, dir);
    }
    if (!essentials::FileSystem::isDirectory(dir)) {
        AlicaEngine::abort("PP: RoleSet directory does not exist: " + dir);
    }

    std::vector<std::string> files = essentials::FileSystem::findAllFiles(dir, ".rset");

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
    bool found = essentials::FileSystem::findFile(this->_basePlanPath, masterplan + ".pml", masterPlanPath);
#ifdef PP_DEBUG
    std::cout << "PP: masterPlanPath: " << masterPlanPath << std::endl;
#endif
    if (!found) {
        AlicaEngine::abort("PP: Cannot find MasterPlan '" + masterplan + "'");
    }
    this->_currentFile = masterPlanPath;
    this->_currentDirectory = essentials::FileSystem::getParent(masterPlanPath);
#ifdef PP_DEBUG
    std::cout << "PP: CurFile: " << this->_currentFile << " CurDir: " << this->_currentDirectory << std::endl;
#endif

    this->_masterPlan = parsePlanFile(masterPlanPath);
    this->_filesParsed.push_back(masterPlanPath);
    parseFileLoop();

    this->_mf->computeReachabilities();
    return this->_masterPlan;
}

void PlanParser::parseFileLoop()
{
    while (this->_filesToParse.size() > 0) {
        string fileToParse = this->_filesToParse.front();
        this->_filesToParse.pop_front();
        this->_currentDirectory = essentials::FileSystem::getParent(fileToParse);
        this->_currentFile = fileToParse;

        if (!essentials::FileSystem::pathExists(fileToParse)) {
            AlicaEngine::abort("PP: Cannot Find referenced file ", fileToParse);
        }
        if (essentials::FileSystem::endsWith(fileToParse, ".pml")) {
            parsePlanFile(fileToParse);
        } else if (essentials::FileSystem::endsWith(fileToParse, ".tsk")) {
            parseTaskFile(fileToParse);
        } else if (essentials::FileSystem::endsWith(fileToParse, ".beh")) {
            parseBehaviourFile(fileToParse);
        } else if (essentials::FileSystem::endsWith(fileToParse, ".pty")) {
            parsePlanTypeFile(fileToParse);
        } else if (essentials::FileSystem::endsWith(fileToParse, ".pp")) {
            parsePlanningProblem(fileToParse);
        } else {
            AlicaEngine::abort("PP: Cannot Parse file", fileToParse);
        }
        _filesParsed.push_back(fileToParse);
    }
    this->_mf->attachPlanReferences();
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
    this->_mf->createPlanningProblem(&doc);
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
    this->_mf->createPlanType(&doc);
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
    this->_mf->createBehaviour(&doc);
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
    this->_mf->createTasks(&doc);
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
        p = this->_mf->createPlan(&doc);
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
    return _mf->getElements();
}

void PlanParser::ignoreMasterPlanId(bool val)
{
    this->_mf->setIgnoreMasterPlanId(val);
}

void PlanParser::setCurrentFile(const std::string& currentFile)
{
    if (currentFile.compare(0, _basePlanPath.size(), _basePlanPath)) {
        this->_currentFile = currentFile.substr(_basePlanPath.size());
    } else if (currentFile.compare(0, _baseRolePath.size(), _baseRolePath)) {
        this->_currentFile = currentFile.substr(_baseRolePath.size());
    } else {
        this->_currentFile = currentFile;
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
            id = stoll(idString);
        } catch (std::exception& e) {
            AlicaEngine::abort("PP: Cannot convert ID to int64_t: " + idString + "\nException: " + e.what());
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
        if (!essentials::FileSystem::endsWith(this->_currentDirectory, "/")) {
            this->_currentDirectory = this->_currentDirectory + "/";
        }
        string path = this->_currentDirectory + locator;
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
        // list<string>::iterator findIterParsed = find(_filesParsed.begin(), _filesParsed.end(), pathNew);
        bool found = false;
        for (auto& it : _filesParsed) {
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

        // list<string>::iterator findIterToParse = find(_filesToParse.begin(), _filesToParse.end(), pathNew);
        if (!found) {
            for (auto& it : _filesToParse) {
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
            _filesToParse.push_back(path);
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
