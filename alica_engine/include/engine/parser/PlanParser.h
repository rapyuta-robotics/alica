#pragma once

#include <iostream>
#include <list>
#include <stdio.h>
#include <exception>
#include <algorithm>
#include <string>

#include <SystemConfig.h>
#include <FileSystem.h>

namespace tinyxml2 {
class XMLElement;
}

namespace alica {

class ModelFactory;
class PlanRepository;
class Plan;
class RoleSet;
class AlicaElement;
class AlicaEngine;

/**
 * The default parser, parsing the XML encoding of an ALICA plan-tree
 */
class PlanParser {
public:
    PlanParser(AlicaEngine* ae, PlanRepository* rep);
    virtual ~PlanParser();

    virtual Plan* parsePlanTree(string masterplan);
    virtual void ignoreMasterPlanId(bool val);
    virtual map<long, AlicaElement*>* getParsedElements();

    string getCurrentFile();
    void setCurrentFile(string currentFile);
    void parseFileLoop();
    RoleSet* parseRoleSet(string roleSetName, string roleSetDir);
    long parserId(tinyxml2::XMLElement* node);

private:
    AlicaEngine* ae;
    supplementary::SystemConfig* sc;
    shared_ptr<ModelFactory> mf;
    PlanRepository* rep;
    Plan* masterPlan;
    string planDir;
    string roleDir;
    string basePlanPath;
    string baseRolePath;
    string currentDirectory;
    string domainConfigFolder;
    string currentFile;
    void parseTaskFile(string currentFile);
    void parseRoleDefFile(string currentFile);
    void parseCapabilityDefFile(string currentFile);
    void parsePlanTypeFile(string currentFile);
    void parseBehaviourFile(string currentFile);
    void parsePlanningProblem(string currentFile);
    Plan* parsePlanFile(string& planFile);
    long fetchId(const string& idString, long id);
    string findDefaultRoleSet(string dir);

    list<string> filesToParse;
    list<string> filesParsed;
};
}  // namespace alica
