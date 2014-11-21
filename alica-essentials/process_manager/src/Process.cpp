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

	/**
	 * This constructor is for processes, which are not self-started by the process manager.
	 * That means, we don't know the start parameters before.
	 * @param pid
	 */
	Process::Process(long pid) :
			pid(pid), state('T')
	{
		std::ifstream statFile("/proc/" + to_string(pid) + "/cmdline", std::ifstream::in);
		string line;
		stringstream ss;
		while (!statFile.eofbit)
		{
			getline(statFile, line, '\0');
			ss << line << " ";
#ifdef PROC_DEBUG
			cout << "Proc: cmdline arg " << line << endl;
#endif
		}
		this->params = ss.str();
	}

	Process::~Process()
	{

	}

	string Process::getRobot()
	{
		return robot;
	}

	void Process::update()
	{
		// TODO: update resource infos over proc-fs
		std::ifstream statFile("/proc/" + to_string(pid) + "/stat", std::ifstream::in);
		string line;
		cout << "Proc: updating pid: " << pid << endl;
		while (getline(statFile, line, ' '))
		{
#ifdef PROC_DEBUG
			cout << "Proc: stat " << line << endl;
#endif
		}
		cout << "Proc: finished updating pid: " << pid << endl;
	}

	string Process::toString()
	{
		stringstream ss;
		ss << "PID: " << pid << " State: " << state << " Params:" << params << endl;
		return ss.str();
	}

} /* namespace supplementary */
