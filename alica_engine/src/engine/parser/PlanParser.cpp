//#define PP_DEBUG
#include "engine/parser/PlanParser.h"
#include "engine/parser/ModelFactory.h"
#include "engine/AlicaEngine.h"
#include "engine/model/Plan.h"

namespace alica {

/**
 * Constructor
 * @param A PlanRepository, in which parsed elements are stored.
 */
PlanParser::PlanParser(PlanRepository* rep) {
    using namespace supplementary;

    this->masterPlan = nullptr;
    this->rep = rep;
    this->mf = shared_ptr<ModelFactory>(new ModelFactory(this, rep));
    this->sc = SystemConfig::getInstance();
    this->domainConfigFolder = this->sc->getConfigPath();

    this->planDir = (*this->sc)["Alica"]->get<string>("Alica.PlanDir", NULL);
    this->roleDir = (*this->sc)["Alica"]->get<string>("Alica.RoleDir", NULL);

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
#ifdef PP_DEBUG
    cout << "PP: basePlanPath: " << basePlanPath << endl;
    cout << "PP: baseRolePath: " << baseRolePath << endl;
#endif
    if (!(supplementary::FileSystem::pathExists(basePlanPath))) {
        AlicaEngine::abort("PP: BasePlanPath does not exists " + basePlanPath);
    }
    if (!(supplementary::FileSystem::pathExists(baseRolePath))) {
        AlicaEngine::abort("PP: BaseRolePath does not exists " + baseRolePath);
    }
}

PlanParser::~PlanParser() {}

/**
 * Parse a roleset
 * @param roleSetName The name of the roleset to parse. May be empty, in which case a default roleset is looked for.
 * @param roleSetDir The relative directory in which to search for a roleset.
 * @return The parsed roleSet
 */
RoleSet* PlanParser::parseRoleSet(string roleSetName, string roleSetDir) {
    using namespace supplementary;

    if (roleSetName.empty()) {
        roleSetName = findDefaultRoleSet(roleSetDir);
    } else {
        if (roleSetDir.find_last_of(FileSystem::PATH_SEPARATOR) != roleSetDir.length() - 1 && roleSetDir.length() > 0) {
            roleSetDir = roleSetDir + FileSystem::PATH_SEPARATOR;
        }
        if (!FileSystem::isPathRooted(roleSetDir)) {
            roleSetName = FileSystem::combinePaths(FileSystem::combinePaths(baseRolePath, roleSetDir), roleSetName);
        } else {
            roleSetName = FileSystem::combinePaths(roleSetDir, roleSetName);
        }
    }

    if (!supplementary::FileSystem::endsWith(roleSetName, ".rset")) {
        roleSetName = roleSetName + ".rset";
    }
    if (!supplementary::FileSystem::pathExists(roleSetName)) {
        AlicaEngine::abort("PP: Cannot find roleset: " + roleSetName);
    }

#ifdef PP_DEBUG
    cout << "PP: Parsing RoleSet " << roleSetName << endl;
#endif

    this->currentDirectory = supplementary::FileSystem::getParent(roleSetName);

    tinyxml2::XMLDocument doc;
    doc.LoadFile(roleSetName.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw new exception();
    }

    RoleSet* r = this->mf->createRoleSet(&doc, this->masterPlan);

    filesParsed.push_back(roleSetName);

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
void PlanParser::parseRoleDefFile(string currentFile) {
#ifdef PP_DEBUG
    cout << "PP: parsing RoleDef file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw new exception();
    }
    this->mf->createRoleDefinitionSet(&doc);
}
void PlanParser::parseCapabilityDefFile(string currentFile) {
#ifdef PP_DEBUG
    cout << "PP: parsing RoleDef file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw new exception();
    }
    this->mf->createCapabilityDefinitionSet(&doc);
}

string PlanParser::findDefaultRoleSet(string dir) {
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
            throw new exception();
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
Plan* PlanParser::parsePlanTree(string masterplan) {
    string masterPlanPath;
    bool found = supplementary::FileSystem::findFile(this->basePlanPath, masterplan + ".pml", masterPlanPath);
#ifdef PP_DEBUG
    cout << "PP: masterPlanPath: " << masterPlanPath << endl;
#endif
    if (!found) {
        AlicaEngine::abort("PP: Cannot find MasterPlan '" + masterplan + "'");
    }
    this->currentFile = masterPlanPath;
    this->currentDirectory = supplementary::FileSystem::getParent(masterPlanPath);
#ifdef PP_DEBUG
    cout << "PP: CurFile: " << this->currentFile << " CurDir: " << this->currentDirectory << endl;
#endif

    this->masterPlan = parsePlanFile(masterPlanPath);
    this->filesParsed.push_back(masterPlanPath);
    parseFileLoop();

    this->mf->computeReachabilities();
    return this->masterPlan;
}

void PlanParser::parseFileLoop() {
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
void PlanParser::parsePlanningProblem(string currentFile) {
#ifdef PP_DEBUG
    cout << "PP: parsing Planning Problem file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw new exception();
    }
    this->mf->createPlanningProblem(&doc);
}

void PlanParser::parsePlanTypeFile(string currentFile) {
#ifdef PP_DEBUG
    cout << "PP: parsing PlanType file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw new exception();
    }
    this->mf->createPlanType(&doc);
}
void PlanParser::parseBehaviourFile(string currentFile) {
#ifdef PP_DEBUG
    cout << "PP: parsing Behaviour file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw new exception();
    }
    this->mf->createBehaviour(&doc);
}

void PlanParser::parseTaskFile(string currentFile) {
#ifdef PP_DEBUG
    cout << "PP: parsing Task file: " << currentFile << endl;
#endif
    tinyxml2::XMLDocument doc;
    doc.LoadFile(currentFile.c_str());
#ifdef PP_DEBUG
    cout << "TASKREPO " << currentFile << endl;
#endif
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw new exception();
    }
    this->mf->createTasks(&doc);
}

Plan* PlanParser::parsePlanFile(string& planFile) {
#ifdef PP_DEBUG
    cout << "PP: parsing Plan file: " << planFile << endl;
#endif

    tinyxml2::XMLDocument doc;
    doc.LoadFile(planFile.c_str());
    if (doc.ErrorID() != tinyxml2::XML_NO_ERROR) {
        cout << "PP: doc.ErrorCode: " << tinyxml2::XMLErrorStr[doc.ErrorID()] << endl;
        throw new exception();
    }
    Plan* p = this->mf->createPlan(&doc);
    return p;
}

/**
 * Provides access to all parsed elements, indexed by their id.
 * @return A shared_ptr<map<long, alica::AlicaElement>
 */
map<long, alica::AlicaElement*>* PlanParser::getParsedElements() {
    return mf->getElements();
}

void PlanParser::ignoreMasterPlanId(bool val) {
    this->mf->setIgnoreMasterPlanId(val);
}

string PlanParser::getCurrentFile() {
    return this->currentFile;
}

void PlanParser::setCurrentFile(string currentFile) {
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
long PlanParser::parserId(tinyxml2::XMLElement* node) {
    long id = -1;
    string idString = "";
    const char* idChar = node->Attribute("id");
    if (idChar)
        idString = idChar;
    if (idString.compare("") != 0) {
        try {
            id = stol(idString);
        } catch (exception& e) {
            AlicaEngine::abort("PP: Cannot convert ID to long: " + idString + " WHAT?? " + e.what());
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
long PlanParser::fetchId(const string& idString, long id) {
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
        temp = realpath(path.c_str(), NULL);
        string pathNew = temp;
        free(temp);
        // This is not very efficient but necessary to keep the paths as they are
        // Here we have to check whether the file has already been parsed / is in the list for toparse files
        // problem is the normalization /home/etc/plans != /home/etc/Misc/../plans
        // list<string>::iterator findIterParsed = find(filesParsed.begin(), filesParsed.end(), pathNew);
        bool found = false;
        for (auto& it : filesParsed) {
            temp2 = realpath(it.c_str(), NULL);
            string pathNew2 = temp2;
            free(temp2);
            if (pathNew2 == pathNew) {
                found = true;
                break;
            }
        }

        // list<string>::iterator findIterToParse = find(filesToParse.begin(), filesToParse.end(), pathNew);
        if (!found) {
            for (auto& it : filesToParse) {
                temp2 = realpath(it.c_str(), NULL);
                string pathNew2 = temp2;
                free(temp2);
                if (pathNew2 == pathNew) {
                    found = true;
                    break;
                }
            }
        }

        if (!found) {
#ifdef PP_DEBUG
            cout << "PP: Adding " + path + " to parse queue " << endl;
#endif
            filesToParse.push_back(path);
        }
    }
    string tokenId = idString.substr(hashPos + 1, idString.length() - hashPos);
    try {
        id = stol(tokenId);
    } catch (exception& e) {
        AlicaEngine::abort("PP: Cannot convert ID to long: " + tokenId + " WHAT?? " + e.what());
    }
    return id;
}
}  // namespace alica
/* namespace Alica */
