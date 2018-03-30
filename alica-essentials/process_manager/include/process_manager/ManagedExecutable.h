#pragma once
//#define MNGD_EXEC_DEBUG

#include "ExecutableMetaData.h"
#include "process_manager/ProcessStats.h"
#include "process_manager/ProcessStat.h"

#include <supplementary/AgentID.h>

#include <ros/console.h>

#include <string>
#include <vector>
#include <chrono>
#include <thread>

namespace supplementary {
class ProcessManager;
class ExecutableMetaData;
class AgentID;

enum RunState { SHOULD_RUN, SHOULDNT_RUN, MANUAL_STARTED };

class ManagedExecutable {
public:
    ManagedExecutable(
            ExecutableMetaData const* const metaExec, long pid, std::string robotName, ProcessManager* procMan);
    virtual ~ManagedExecutable();
    void queue4Update(long pid);
    void update(unsigned long long cpuDelta);
    void report(process_manager::ProcessStats& psts, const AgentID* robotId);
    void changeDesiredState(bool shouldRun, int paramSetId);
    void changeDesiredLogPublishingState(bool shouldPublish);
    void startProcess(std::vector<char*>& params);
    void startProcess();
    void startPublishingLogs();
    void stopPublishingLogs();
    void publishLogFile(std::string logFileName, ros::console::levels::Level logLevel);

    static long kernelPageSize; /* < in bytes */

    ExecutableMetaData const* const metaExec;

private:
    // Information about the managed process (updated continuously)
    long managedPid;
    std::vector<char const*> runningParams;
    int runningParamSet;
    char state;  // The process state (zombie, running, etc)
    unsigned long long lastUTime;
    unsigned long long lastSTime;
    unsigned long long currentUTime;
    unsigned long long currentSTime;
    unsigned long long starttime;
    unsigned short cpu;
    long int memory;
    std::string robotEnvVariable;

    // log publishing
    bool publishing;
    bool shouldPublish;
    std::string stdLogFileName;
    std::string errLogFileName;
    std::thread stdLogPublisher;
    std::thread errLogPublisher;

    std::chrono::time_point<std::chrono::steady_clock> lastTimeTried;
    RunState desiredRunState;
    bool need2ReadParams;
    int desiredParamSet;
    std::vector<long>
            queuedPids4Update; /* < a list of PIDs, which match this managed executable (should be only one, normally)*/
    ProcessManager* procMan;

    void updateStats(unsigned long long cpuDelta, bool isNew = false);
    void readProcParams(std::string procPidString);
    void printStats();
    void killQueuedProcesses();
    void readParams(long pid);
    void clear();
};

} /* namespace supplementary */
