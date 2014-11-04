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

#include "Process.h"

using namespace std;

namespace supplementary
{

	class ManagedExecutable
	{
	public:
		ManagedExecutable(short id, string executable, string defaultParams);
		virtual ~ManagedExecutable();
		const string& getExecutable() const;
		void queue4Update(long pid);
		void update();
	private:
		short id;
		string executable;
		string defaultParams;
		map<long, Process> processes;
		vector<long> queuedPids4Update;
	};

} /* namespace supplementary */

#endif /* MANAGEDEXECUTABLE_H_ */
