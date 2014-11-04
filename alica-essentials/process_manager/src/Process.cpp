/*
 * Process.cpp
 *
 *  Created on: Nov 1, 2014
 *      Author: Stephan Opfer
 */

#include "Process.h"

namespace supplementary
{

	Process::Process(long pid, string params) :
			pid(pid), params(params), state('T')
	{

	}

	Process::~Process()
	{

	}

	void Process::update()
	{
		// TODO: update resource infos over proc-fs
		std::ifstream statFile ("/proc/" + to_string(pid) + "/stat", std::ifstream::in);
		string line;

		while(!statFile.eofbit) {
			getline(statFile, line, '\0');
#ifdef PROC_DEBUG
			cout << "Proc: " << line << endl;
#endif
		}
	}

	string Process::toString()
	{
		stringstream ss;
		ss << "PID: " << pid << " State: " << state << " Params:" << params << endl;
		return ss.str();
	}

} /* namespace supplementary */
