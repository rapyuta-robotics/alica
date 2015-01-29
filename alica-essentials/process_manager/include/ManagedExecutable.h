/*
 * ManagedExecutable.h
 *
 *  Created on: Nov 4, 2014
 *      Author: Stephan Opfer
 */

#ifndef MANAGEDEXECUTABLE_H_
#define MANAGEDEXECUTABLE_H_

#define MGND_EXEC_DEBUG

#include <string>
#include <vector>
#include <chrono>

using namespace std;

namespace supplementary
{

	class ManagedExecutable
	{
	public:
		ManagedExecutable(string execName, int execid, long pid);
		virtual ~ManagedExecutable();
		string getExecutable() const;
		void queue4Update(long pid);
		void update();
		void changeDesiredState(bool shouldRun);
		void startProcess (char* const* params);
		void startProcess ();
		bool stopProcess ();

		static const long NOTHING_MANAGED = -1;
		static const char UNDEFINED = 'U';
		static long kernelPageSize; // in bytes

	private:
		// General information (fix for object life time)
		int id;
		string executable;
		char ** defaultParams;

		// Information about the managed process (updated continuously)
		long managedPid;
		string params;
		char state; // The process state (zombie, running, etc)
		unsigned long utime;
		unsigned long stime;
		long int cutime;
		long int cstime;
		unsigned long long starttime;
		long int memory;


		chrono::time_point<chrono::steady_clock> lastCommandTime;
		bool shouldRun;
		char ** desiredParams;
		vector<long> queuedPids4Update; /* < a list of PIDs, which match this managed executable (should be only one, normally)*/

		void updateStats(bool readParams = false);
		void printStats();
		void killOtherProcesses();
		void readParams(long pid);
		void clear();

	};

} /* namespace supplementary */

#endif /* MANAGEDEXECUTABLE_H_ */
