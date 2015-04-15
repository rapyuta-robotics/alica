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

} /* namespace rqt_pm_control */
