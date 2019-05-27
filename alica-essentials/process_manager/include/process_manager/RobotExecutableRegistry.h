#pragma once

#include <SystemConfig.h>
#include <essentials/Identifier.h>
#include <essentials/IDManager.h>

#include <map>
#include <stdint.h>
#include <string>
#include <vector>

namespace essentials
{

class RobotMetaData;
class ExecutableMetaData;

/**
 * The RobotExecutableRegistry help the process manager and its
 * control GUI to remember/manage the names and IDs of robots and executables.
 */
class RobotExecutableRegistry
{
public:
    static RobotExecutableRegistry* get();
    const std::map<const essentials::Identifier*, RobotMetaData*, essentials::IdentifierComparator>& getRobots() const;
    void addRobot(std::string agentName, const essentials::Identifier* agentID);
    const essentials::Identifier* addRobot(std::string agentName);
    std::string addRobot(const essentials::Identifier* agentID);
    const essentials::Identifier* getRobotId(const std::string& agentName) const;
    const essentials::Identifier* getRobotId(const std::vector<uint8_t>& idVector);
    const essentials::Identifier* getRobotId(const std::vector<uint8_t>& idVector, std::string& robotName);
    bool getRobotName(const essentials::Identifier* agentID, std::string& robotName);
    bool robotExists(const essentials::Identifier* agentID);
    bool robotExists(std::string agentName);
    void setInterpreters(std::vector<std::string> interpreter);
    bool isKnownInterpreter(std::string const& cmdLinePart);

    std::map<std::string, std::vector<std::pair<int, int>>> const* const getBundlesMap();
    ExecutableMetaData const* const getExecutable(std::string execName) const;
    ExecutableMetaData const* const getExecutable(int execId) const;
    const std::vector<ExecutableMetaData*>& getExecutables() const;
    int addExecutable(std::string execName);
    bool getExecutableId(std::vector<std::string>& splittedCmdLine, int& execId);
    bool getExecutableIdByExecName(std::string execName, int& execId);
    bool getExecutableName(int execId, std::string& execName);
    bool executableExists(int execId);
    bool executableExists(std::string execName);

  private:
    RobotExecutableRegistry();
    virtual ~RobotExecutableRegistry();

    std::map<const essentials::Identifier*, RobotMetaData*, essentials::IdentifierComparator> robotMap;
    std::vector<ExecutableMetaData*> executableList;
    std::vector<std::string> interpreter;
    std::map<std::string, std::vector<std::pair<int, int>>> bundlesMap;
    essentials::SystemConfig* sc;
    essentials::IDManager* agentIDManager;
};

} /* namespace  essentials */
