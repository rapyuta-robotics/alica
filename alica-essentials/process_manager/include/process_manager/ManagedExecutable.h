/*
 * ManagedExecutable.h
 *
 *  Created on: Nov 4, 2014
 *      Author: Stephan Opfer
 */

#ifndef MANAGEDEXECUTABLE_H_
#define MANAGEDEXECUTABLE_H_

//#define MNGD_EXEC_DEBUG

#include <string>
#include <vector>
#include <chrono>
#include "ExecutableMetaData.h"
#include "process_manager/ProcessStats.h"
#include "process_manager/ProcessStat.h"
 #include <ros/console.h>
#include <thread>

using namespace std;

namespace supplementary
{
	class ProcessManager;
	class ExecutableMetaData;

	enum RunState { SHOULD_RUN, SHOULDNT_RUN, MANUAL_STARTED};

	class ManagedExecutable
	{
	public:
		ManagedExecutable(ExecutableMetaData const * const metaExec, long pid, string robotName, ProcessManager* procMan);
		virtual ~ManagedExecutable();
		void queue4Update(long pid);
		void update(unsigned long long cpuDelta);
		void report(process_manager::ProcessStats& psts, int robotId);
		void changeDesiredState(bool shouldRun, int paramSetId);
		void changeDesiredLogPublishingState(bool shouldPublish);
		void startProcess (vector<char*> & params);
		void startProcess ();
		void startPublishingLogs();
		void stopPublishingLogs();
		void publishLogFile(string logFileName, ros::console::levels::Level logLevel);

		static long kernelPageSize; /* < in bytes */

		ExecutableMetaData const * const metaExec;
	private:

		// Information about the managed process (updated continuously)
		long managedPid;
		vector<char const *> runningParams;
		int runningParamSet;
		char state; // The process state (zombie, running, etc)
		unsigned long long lastUTime;
		unsigned long long lastSTime;
		unsigned long long currentUTime;
		unsigned long long currentSTime;
		unsigned long long starttime;
		unsigned short cpu;
		long int memory;
		string robotEnvVariable;

		// log publishing
		bool publishing;
		bool shouldPublish;
		string stdLogFileName;
		string errLogFileName;
		thread stdLogPublisher;
		thread errLogPublisher;

		chrono::time_point<chrono::steady_clock> lastTimeTried;
		RunState desiredRunState;
		bool need2ReadParams;
		int desiredParamSet;
		vector<long> queuedPids4Update; /* < a list of PIDs, which match this managed executable (should be only one, normally)*/
		ProcessManager* procMan;

		void updateStats(unsigned long long cpuDelta, bool isNew = false);
		void readProcParams(string procPidString);
		void printStats();
		void killQueuedProcesses();
		void readParams(long pid);
		void clear();

	};

} /* namespace supplementary */

#endif /* MANAGEDEXECUTABLE_H_ */
