#pragma once

#include <algorithm>
#include <exception>
#include <iostream>
#include <list>
#include <stdio.h>
#include <string>
#include <vector>

#include "engine/parser/ModelFactory.h"
#include <FileSystem.h>
#include <SystemConfig.h>

namespace alica
{

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
    bool isUniqueElement(int64_t elementId);

    const std::string& getCurrentFile() const { return _currentFile; }
    void setCurrentFile(const std::string& currentFile);
    void parseFileLoop();
    const RoleSet* parseRoleSet(std::string roleSetName);
    int64_t parserId(tinyxml2::XMLElement* node);

private:
    ModelFactory _mf;
    PlanRepository* _rep;
    Plan* _masterPlan;
    std::string _planDir;
    std::string _roleDir;
    std::string _basePlanPath;
    std::string _baseRolePath;
    std::string _currentDirectory;
    std::string _currentFile;
    std::vector<std::string> _filesToParse;
    std::vector<std::string> _filesParsed;

    void parseFile(const std::string& currentFile, tinyxml2::XMLDocument& doc);
    void parseTaskFile(const std::string& currentFile);
    void parseRoleDefFile(const std::string& currentFile);
    void parseCapabilityDefFile(const std::string& currentFile);
    void parsePlanTypeFile(const std::string& currentFile);
    void parseBehaviourFile(const std::string& currentFile);
    void parsePlanningProblem(const std::string& currentFile);
    Plan* parsePlanFile(const std::string& planFile);
    int64_t fetchId(const std::string& idString, int64_t id);
    std::string findDefaultRoleSet(std::string dir);
};
} // namespace alica
