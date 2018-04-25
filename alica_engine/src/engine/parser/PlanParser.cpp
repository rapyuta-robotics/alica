#define PP_DEBUG
#include "engine/parser/PlanParser.h"
#include "engine/AlicaEngine.h"
#include "engine/model/Plan.h"
#include "engine/parser/ModelFactory.h"

namespace alica
{

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

    this->planDir = (*this->sc)["Alica"]->get<string>("Alica.PlanDir", NULL);
    this->roleDir = (*this->sc)["Alica"]->get<string>("Alica.RoleDir", NULL);
    this->taskDir = (*this->sc)["Alica"]->get<string>("Alica.TaskDir", NULL);

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

#ifdef PP_DEBUG
    std::cout << "PP: basePlanPath: " << basePlanPath << endl;
    std::cout << "PP: baseRolePath: " << baseRolePath << endl;
    std::cout << "PP: baseTaskPath: " << baseTaskPath << endl;
#endif
    if (!(supplementary::FileSystem::pathExists(basePlanPath))) {
        AlicaEngine::abort("PP: BasePlanPath does not exists " + basePlanPath);
    }
    if (!(supplementary::FileSystem::pathExists(baseRolePath))) {
        AlicaEngine::abort("PP: BaseRolePath does not exists " + baseRolePath);
    }
}

/**
 * Parse a roleset
 * @param roleSetName The name of the roleset to parse. May be empty, in which case a default roleset is looked for.
 * @param roleSetDir The relative directory in which to search for a roleset.
 * @return The parsed roleSet
 */
RoleSet* PlanParser::parseRoleSet(string roleSetName, string roleSetDir)
{
#ifdef PP_DEBUG
    std::cout << "PP: parseRoleSet(): roleSetName: " << roleSetName << " roleSetDir: " << roleSetDir << std::endl;
#endif
    if (roleSetName.empty()) {
        roleSetName = findDefaultRoleSet(roleSetDir);
    } else {
        if (roleSetDir.find_last_of(supplementary::FileSystem::PATH_SEPARATOR) != roleSetDir.length() - 1 && roleSetDir.length() > 0) {
            roleSetDir = roleSetDir + supplementary::FileSystem::PATH_SEPARATOR;
        }
        if (!supplementary::FileSystem::isPathRooted(roleSetDir)) {
            roleSetName = supplementary::FileSystem::combinePaths(supplementary::FileSystem::combinePaths(baseRolePath, roleSetDir), roleSetName);
        } else {
            roleSetName = supplementary::FileSystem::combinePaths(roleSetDir, roleSetName);
        }
    }

    if (!supplementary::FileSystem::endsWith(roleSetName, ".rset")) {
        roleSetName = roleSetName + ".rset";
    }
    if (!supplementary::FileSystem::pathExists(roleSetName)) {
        AlicaEngine::abort("PP: Cannot find roleset: " + roleSetName);
    }

#ifdef PP_DEBUG
    std::cout << "PP: Parsing RoleSet: " << roleSetName << std::endl;
#endif

    this->currentDirectory = supplementary::FileSystem::getParent(roleSetName);

    tinyxml2::XMLDocument doc;
    doc.LoadFile(roleSetName.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw exception();
    }

    RoleSet* r = this->mf->createRoleSet(&doc, this->masterPlan);

    filesParsed.push_back(roleSetName);

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
void PlanParser::parseRoleDefFile(string currentFile)
{
#ifdef PP_DEBUG
    cout << "PP: Parsing RoleDef file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw exception();
    }
    this->mf->createRoleDefinitionSet(&doc);
}
void PlanParser::parseCapabilityDefFile(string currentFile)
{
#ifdef PP_DEBUG
    cout << "PP: Parsing RoleDef file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw exception();
    }
    this->mf->createCapabilityDefinitionSet(&doc);
}

string PlanParser::findDefaultRoleSet(string dir)
{
    if (!supplementary::FileSystem::isPathRooted(dir)) {
        dir = supplementary::FileSystem::combinePaths(this->baseRolePath, dir);
    }
    if (!supplementary::FileSystem::isDirectory(dir)) {
        AlicaEngine::abort("PP: RoleSet directory does not exist: " + dir);
    }

    vector<string> files = supplementary::FileSystem::findAllFiles(dir, ".rset");

    for (string s : files) {
        tinyxml2::XMLDocument doc;
        doc.LoadFile(s.c_str());
        if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
            cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
            throw exception();
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

/**
 * Parses a plan tree
 * @param masterplan The name of the top-level plan
 * @return The top-level plan
 */
Plan* PlanParser::parsePlanTree(const string& masterplan)
{
    string masterPlanPath;
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
    std::cout << "PP: CurFile: " << this->currentFile << std::endl << "    CurDir: " << this->currentDirectory << std::endl;
#endif

    this->filesParsed.push_back(masterPlanPath);
    this->masterPlan = parsePlanFile(masterPlanPath);
    parseFileLoop();

    this->mf->computeReachabilities();
    return this->masterPlan;
}

void PlanParser::parseFileLoop()
{
    while (!this->filesToParse.empty()) {
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

void PlanParser::parsePlanTypeFile(string currentFile)
{
#ifdef PP_DEBUG
    cout << "PP: Parsing PlanType file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw exception();
    }
    this->mf->createPlanType(&doc);
}
void PlanParser::parseBehaviourFile(string currentFile)
{
#ifdef PP_DEBUG
    cout << "PP: Parsing Behaviour file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw exception();
    }
    this->mf->createBehaviour(&doc);
}

void PlanParser::parseTaskFile(string currentFile)
{
#ifdef PP_DEBUG
    cout << "PP: Parsing Task file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw exception();
    }
    this->mf->createTasks(&doc);
}

Plan* PlanParser::parsePlanFile(string& planFile)
{
#ifdef PP_DEBUG
    cout << "PP: Parsing Plan file: " << planFile << endl;
#endif

    tinyxml2::XMLDocument doc;
    doc.LoadFile(planFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw exception();
    }
    Plan* p = this->mf->createPlan(&doc);
    return p;
}

/**
 * Provides access to all parsed elements, indexed by their id.
 * @return A shared_ptr<map<long, alica::AlicaElement>
 */
map<long, alica::AlicaElement*>* PlanParser::getParsedElements()
{
    return mf->getElements();
}

string PlanParser::getCurrentFile()
{
    return this->currentFile;
}

/**
 * parse id of the plan
 * @param node the given xml node
 * @return id: returns the id from the plan
 */
long PlanParser::parserId(tinyxml2::XMLElement* node)
{
    long id = -1;
    string idString;
    const char* idChar = node->Attribute("id");
    if (idChar)
        idString = idChar;
    if (!idString.empty()) {
        try {
            id = stol(idString);
        } catch (exception& e) {
            AlicaEngine::abort("PP: Cannot convert ID to long: " + idString + " WHAT?? " + e.what());
        }
        return id;
    } else {
        idChar = node->Attribute("href");
        if (idChar)
            idString = idChar;
        if (!idString.empty()) {
            id = fetchId(idString);
            return id;
        } else {
            const tinyxml2::XMLNode* currNode = node->FirstChild();
            while (currNode) {
                const tinyxml2::XMLText* textNode = currNode->ToText();
                if (textNode) {
                    id = fetchId(textNode->Value());
                    return id;
                }

                currNode = currNode->NextSibling();
            }
        }
    }

    cerr << "Cannot resolve remote reference!\nAttributes of node in question are:" << endl;
    const tinyxml2::XMLAttribute* curAttribute = node->FirstAttribute();
    while (curAttribute) {
        cout << curAttribute->Name() << " : " << curAttribute->Value() << endl;
        curAttribute = curAttribute->Next();
    }
    AlicaEngine::abort("PP: Couldn't resolve remote reference: " + string(node->Name()));
    return -1;
}

/**
 * Helper
 * @param idString is a String that have to be converted in a long
 * @param id the id
 */
long PlanParser::fetchId(const string& idString)
{
    unsigned long hashPos = idString.find_first_of('#');
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
        } else{
            std::cout << "PP: Unknown locator ending: " << locator << std::endl;
        }

        if (std::find(std::begin(filesParsed), std::end(filesParsed), path) == std::end(filesParsed) &&
                std::find(std::begin(filesToParse), std::end(filesToParse), path) == std::end(filesToParse)) {
#ifdef PP_DEBUG
            cout << "PP: Adding " + path + " to parse queue " << endl;
#endif
            filesToParse.push_back(path);
        }
    }
    string tokenId = idString.substr(hashPos + 1, idString.length() - hashPos);
    long id = -1;
    try {
        id = stol(tokenId);
    } catch (exception& e) {
        AlicaEngine::abort("PP: Cannot convert ID to long: " + tokenId + " WHAT?? " + e.what());
    }
    return id;
}
} // namespace alica
/* namespace Alica */
