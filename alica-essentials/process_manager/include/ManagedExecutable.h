/*
 * ManagedExecutable.h
 *
 *  Created on: Nov 4, 2014
 *      Author: Stephan Opfer
 */

#ifndef MANAGEDEXECUTABLE_H_
#define MANAGEDEXECUTABLE_H_

#define MGND_EXEC_DEBUG

#include <map>
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <sstream>
#include <vector>
#include <fstream>

using namespace std;

namespace supplementary
{

	class ManagedExecutable
	{
	public:
		ManagedExecutable(short id, const char* executable, vector<string> defaultStrParams);
		ManagedExecutable(string execName, uint8_t execid, long pid);
		virtual ~ManagedExecutable();
		string getExecutable() const;
		void queue4Update(long pid);
		void update();
		void startProcess (char* const* params);
		void startProcess ();
		bool stopProcess ();

		static const long NOTHING_MANAGED = -1;
		static const char UNDEFINED = 'U';

	private:
		// General information (fix for object life time)
		short id;
		const char* executable;
		char ** defaultParams;

		// Information about the managed process (updated continuously)
		long managedPid;
		string params;
		char state; // The process state (zombie, running, etc)
		// TODO: Add and update statistic fields about CPU and Memory

		vector<long> queuedPids4Update; // a list of PIDs, which match this managed executable (should be only one, normally)

		void updateStats(bool readParams = false);
		void killOtherProcesses();
		void readParams(long pid);
		void clear();
	};

} /* namespace supplementary */

#endif /* MANAGEDEXECUTABLE_H_ */
