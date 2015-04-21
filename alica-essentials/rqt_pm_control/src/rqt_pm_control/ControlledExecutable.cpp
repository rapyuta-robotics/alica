/*
 * ControlledExecutable.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_pm_control/ControlledExecutable.h"

namespace rqt_pm_control
{

	ControlledExecutable::ControlledExecutable(string execName, int execId, string mode, vector<char*> defaultParams, string absExecName) :
			ExecutableMetaData(execName, execId, mode, defaultParams, absExecName), memory(0), state('X'), cpu(0)
	{

	}

	ControlledExecutable::~ControlledExecutable()
	{

	}

	/**
	 * Updates the informations about the process with the given information.
	 * @param ps The given information about the process.
	 */
	void ControlledExecutable::handleStat(process_manager::ProcessStat ps)
	{
		this->timeLastMsgReceived = chrono::system_clock::now();
		cout << "ControlledExecutable: Set last message time to " << this->timeLastMsgReceived.time_since_epoch().count() << endl;

		this->cpu = ps.cpu;
		this->memory = ps.mem;
		this->state = ps.state;
		// TODO: Maybe transmit parameters?
	}

} /* namespace rqt_pm_control */
