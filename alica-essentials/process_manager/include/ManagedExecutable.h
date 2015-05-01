/*
 * ManagedExecutable.h
 *
 *  Created on: Nov 4, 2014
 *      Author: Stephan Opfer
 */

#ifndef MANAGEDEXECUTABLE_H_
#define MANAGEDEXECUTABLE_H_

//#define MGND_EXEC_DEBUG

#include <string>
#include <vector>
#include <chrono>
#include "ExecutableMetaData.h"
#include "process_manager/ProcessStats.h"
#include "process_manager/ProcessStat.h"

using namespace std;

namespace supplementary
{

	class ManagedExecutable : public ExecutableMetaData
	{
	public:
		ManagedExecutable(string execName, int execid, long pid, string mode, vector<char*> defaultParams, string absExecName, string robotName);
		virtual ~ManagedExecutable();
		void queue4Update(long pid);
		void update(unsigned long long cpuDelta);
		void report(process_manager::ProcessStats& psts, int robotId);
		void changeDesiredState(bool shouldRun);
		void startProcess (vector<char*>& params);
		void startProcess ();
		bool stopProcess ();

		static const long NOTHING_MANAGED = -1;
		static long kernelPageSize; /* < in bytes */

	private:

		// Information about the managed process (updated continuously)
		long managedPid;
		char** params;
		char state; // The process state (zombie, running, etc)
		unsigned long long lastUTime;
		unsigned long long lastSTime;
		unsigned long long currentUTime;
		unsigned long long currentSTime;
		unsigned long long starttime;
		unsigned short cpu;
		long int memory;
		string robotEnvVariable;


		chrono::time_point<chrono::steady_clock> lastTimeTried;
		bool shouldRun;
		char ** desiredParams;
		vector<long> queuedPids4Update; /* < a list of PIDs, which match this managed executable (should be only one, normally)*/

		void updateStats(unsigned long long cpuDelta, bool isNew = false, bool readParams = false);
		void readProcParams(string procPidString);
		void printStats();
		void killQueuedProcesses();
		void readParams(long pid);
		void clear();

	};

} /* namespace supplementary */

#endif /* MANAGEDEXECUTABLE_H_ */
