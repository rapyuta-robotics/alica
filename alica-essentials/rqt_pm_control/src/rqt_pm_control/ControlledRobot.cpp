/*
 * ControlledRobot.cpp
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#include <rqt_pm_control/ControlledRobot.h>
#include <rqt_pm_control/ControlledExecutable.h>
#include <RobotExecutableRegistry.h>

namespace rqt_pm_control
{

	ControlledRobot::ControlledRobot(string robotName, int robotId) :
			RobotMetaData(robotName, robotId)
	{
		// TODO Auto-generated constructor stub

	}

	ControlledRobot::~ControlledRobot()
	{
		// TODO Auto-generated destructor stub
	}

	void ControlledRobot::handleProcessStat(process_manager::ProcessStat ps, supplementary::RobotExecutableRegistry* pmRegistry)
	{
		this->timeLastMsgReceived = chrono::system_clock::now();
		cout << "ControlledRobot: Set last message time to " << this->timeLastMsgReceived.time_since_epoch().count() << endl;

		ControlledExecutable* controlledExec;
		auto controlledExecEntry = this->controlledExecMap.find(ps.processKey);
		if (controlledExecEntry != this->controlledExecMap.end())
		{ // executable is already known
			controlledExec = controlledExecEntry->second;
		}
		else
		{ // executable is unknown
			supplementary::ExecutableMetaData const * const execMetaData = pmRegistry->getExecutable(ps.processKey);
			if (execMetaData != nullptr)
			{
				cout << "ControlledRobot: Create new ControlledExecutable with ID " << ps.processKey << " and executable name " << execMetaData->name << "!" << endl;
				controlledExec = new ControlledExecutable(execMetaData->name, ps.processKey,
														  execMetaData->mode, execMetaData->defaultParams,
														  execMetaData->absExecName);
				this->controlledExecMap.emplace(ps.processKey, controlledExec);
			}
			else
			{
				cerr << "ControlledRobot: Received processStat for unknown executable with process key " << ps.processKey << endl;
				return;
			}
		}

		// update the statistics of the ControlledExecutable
		controlledExec->handleStat(ps);
	}

} /* namespace rqt_pm_control */
