/*
 * ControlledExecutable.cpp
 *
 *  Created on: Mar 2, 2015
 *      Author: Stephan Opfer
 */

#include "ControlledExecutable.h"

namespace rqt_pm_control
{

	ControlledExecutable::ControlledExecutable(string execName, int execId, string mode, vector<char*> defaultParams) :
			ExecutableMetaData(execName, execId, mode, defaultParams), memory(0), state('X'), cpu(0)
	{

	}

	ControlledExecutable::~ControlledExecutable()
	{

	}

} /* namespace rqt_pm_control */
