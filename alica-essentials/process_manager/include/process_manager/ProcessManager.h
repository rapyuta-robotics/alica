#pragma once

#define PM_DEBUG // for toggling debug output

#include "process_manager/ProcessCommand.h"
#include "process_manager/ProcessStat.h"
#include "process_manager/ProcessStats.h"

#include <SystemConfig.h>
#include <essentials/AgentID.h>
#include <essentials/AgentIDFactory.h>

#include <chrono>
#include <ros/ros.h>

namespace std
{
class thread;
}

namespace essentials
{

class ManagedRobot;
class ManagedExecutable;
class RobotMetaData;
class ExecutableMetaData;
class RobotExecutableRegistry;

class ProcessManager
{
  public:
    ProcessManager(int argc, char** argv);
    virtual ~ProcessManager();
    void start();
    bool isRunning();

    bool selfCheck();
    void initCommunication(int argc, char** argv);
    bool isKnownInterpreter(std::string const& cmdLinePart);
    std::vector<std::string> splitCmdLine(std::string cmdLine);

    static void pmSigintHandler(int sig);
    static void pmSigchildHandler(int sig);
    static std::string getCmdLine(const char* pid);
    static int numCPUs;  /* < including hyper threading cores */
    static bool running; /* < has to be static, to be changeable within ProcessManager::pmSignintHandler() */

  private:
    const essentials::AgentID* ownId;
    bool simMode;
    std::map<const essentials::AgentID*, ManagedRobot*, essentials::AgentIDComparator> robotMap;
    RobotExecutableRegistry* pmRegistry;
    std::vector<std::string> interpreters;
    unsigned long long lastTotalCPUTime;
    unsigned long long currentTotalCPUTime;

    ros::NodeHandle* rosNode;
    ros::AsyncSpinner* spinner;
    ros::Subscriber processCommandSub;
    std::string processCmdTopic;
    ros::Publisher processStatePub;
    std::string processStatsTopic;

    std::string getRobotEnvironmentVariable(std::string processId);
    void updateTotalCPUTimes();
    void handleProcessCommand(process_manager::ProcessCommandPtr pc);
    void changeDesiredProcessStates(process_manager::ProcessCommandPtr pc, bool shouldRun);
    void changeLogPublishing(process_manager::ProcessCommandPtr pc, bool shouldPublish);
    std::thread* mainThread;
    std::chrono::microseconds iterationTime;

    void run();
    void searchProcFS();
    void update(unsigned long long cpuDelta);
    void report();
};

} // namespace  essentials
