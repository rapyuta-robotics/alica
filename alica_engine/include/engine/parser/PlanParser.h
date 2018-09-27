#pragma once

#include <algorithm>
#include <exception>
#include <iostream>
#include <list>
#include <stdio.h>
#include <string>

#include <FileSystem.h>
#include <SystemConfig.h>

namespace tinyxml2
{
class XMLElement;
}

namespace alica
{

class ModelFactory;
class PlanRepository;
class Plan;
class RoleSet;
class AlicaElement;

/**
 * The default parser, parsing the XML encoding of an ALICA plan-tree
 */
class PlanParser
{
  public:
    PlanParser(PlanRepository* rep);
    ~PlanParser();

    const Plan* parsePlanTree(const std::string& masterplan);
    void ignoreMasterPlanId(bool val);
    std::map<int64_t, AlicaElement*>* getParsedElements();

    const std::string& getCurrentFile() const { return currentFile; }
    void setCurrentFile(const std::string& currentFile);
    void parseFileLoop();
    const RoleSet* parseRoleSet(std::string roleSetName, std::string roleSetDir);
    int64_t parserId(tinyxml2::XMLElement* node);

  private:
    supplementary::SystemConfig* sc;
    std::shared_ptr<ModelFactory> mf;
    PlanRepository* rep;
    Plan* masterPlan;

    std::string planDir;
    std::string roleDir;
    std::string taskDir;
    std::string basePlanPath;
    std::string baseRolePath;
    std::string baseTaskPath;
    std::string currentDirectory;
    std::string domainConfigFolder;
    std::string currentFile;
    void parseTaskFile(const std::string& currentFile);
    void parseRoleDefFile(const std::string& currentFile);
    void parseCapabilityDefFile(const std::string& currentFile);
    void parsePlanTypeFile(const std::string& currentFile);
    void parseBehaviourFile(const std::string& currentFile);
    Plan* parsePlanFile(const std::string& planFile);
    int64_t fetchId(const std::string& idString, int64_t id);
    std::string findDefaultRoleSet(std::string dir);

    std::list<std::string> filesToParse;
    std::list<std::string> filesParsed;
};
} // namespace alica
