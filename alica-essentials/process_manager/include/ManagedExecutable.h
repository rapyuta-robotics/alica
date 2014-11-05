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

#include "Process.h"

using namespace std;

namespace supplementary
{

	class ManagedExecutable
	{
	public:
		ManagedExecutable(short id, const char* executable, vector<string> defaultStrParams);
		virtual ~ManagedExecutable();
		string getExecutable() const;
		void queue4Update(long pid);
		void update();
		void startProcess (char* const* params);
		void startProcess ();
	private:
		short id;
		const char* executable;
		char ** defaultParams;
		map<long, Process*> processes;
		vector<long> queuedPids4Update;
	};

} /* namespace supplementary */

#endif /* MANAGEDEXECUTABLE_H_ */
