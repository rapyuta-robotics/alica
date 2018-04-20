#pragma once

#include <supplementary/AgentID.h>

#include <vector>
#include <string>
#include <map>
#include <stdint.h>

namespace supplementary {

class AgentIDFactory;
class AgentID;
class AgentIDManager;
class RobotMetaData;
class ExecutableMetaData;
class SystemConfig;

/**
 * The RobotExecutableRegistry help the process manager and its
 * control GUI to remember/manage the names and IDs of robots and executables.
 */
class RobotExecutableRegistry {
public:
    static RobotExecutableRegistry* get();
    const std::map<const AgentID*, RobotMetaData*, supplementary::AgentIDComparator>& getRobots() const;
    void addRobot(std::string agentName, const AgentID* agentID);
    const AgentID* addRobot(std::string agentName);
    std::string addRobot(const AgentID* agentID);
    const AgentID* getRobotId(std::string agentName);
    const AgentID* getRobotId(const std::vector<uint8_t>& idVector);
    const AgentID* getRobotId(std::vector<uint8_t>& idVector, std::string& robotName);
    bool getRobotName(const AgentID* agentID, std::string& robotName);
    bool robotExists(const AgentID* agentID);
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

    std::map<const AgentID*, RobotMetaData*, supplementary::AgentIDComparator> robotMap;
    std::vector<ExecutableMetaData*> executableList;
    std::vector<std::string> interpreter;
    std::map<std::string, std::vector<std::pair<int, int>>> bundlesMap;
    SystemConfig* sc;
    supplementary::AgentIDManager* agentIDManager;
};

} /* namespace supplementary */
