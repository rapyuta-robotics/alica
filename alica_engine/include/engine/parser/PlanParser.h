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

/**
 * The default parser, parsing the XML encoding of an ALICA plan-tree
 */
class PlanParser {
public:
    PlanParser(PlanRepository* rep);
    virtual ~PlanParser();

    virtual const Plan* parsePlanTree(const std::string& masterplan);
    virtual void ignoreMasterPlanId(bool val);
    virtual map<int64_t, AlicaElement*>* getParsedElements();

    std::string getCurrentFile();
    void setCurrentFile(string currentFile);
    void parseFileLoop();
    const RoleSet* parseRoleSet(string roleSetName, string roleSetDir);
    int64_t parserId(tinyxml2::XMLElement* node);

private:
    supplementary::SystemConfig* sc;
    shared_ptr<ModelFactory> mf;
    PlanRepository* rep;
    Plan* masterPlan;
    std::string planDir;
    std::string roleDir;
    std::string basePlanPath;
    std::string baseRolePath;
    std::string currentDirectory;
    std::string domainConfigFolder;
    std::string currentFile;
    void parseTaskFile(string currentFile);
    void parseRoleDefFile(string currentFile);
    void parseCapabilityDefFile(string currentFile);
    void parsePlanTypeFile(string currentFile);
    void parseBehaviourFile(string currentFile);
    void parsePlanningProblem(string currentFile);
    Plan* parsePlanFile(const std::string& planFile);
    int64_t fetchId(const std::string& idString, int64_t id);
    std::string findDefaultRoleSet(string dir);

    std::list<std::string> filesToParse;
    std::list<std::string> filesParsed;
};
}  // namespace alica
