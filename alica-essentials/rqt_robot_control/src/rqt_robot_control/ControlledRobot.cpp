/*
 * ControlledRobot.cpp
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_robot_control/ControlledRobot.h"
#include "rqt_robot_control/RobotsControl.h"

#include <RobotExecutableRegistry.h>
#include <limits.h>
#include <ros/ros.h>

namespace rqt_robot_control
{
	ControlledRobot::ControlledRobot(string robotName, int robotId, RobotsControl* parentRobotsControl) :
			RobotMetaData(robotName, robotId), parentRobotsControl(parentRobotsControl)
	{
		//TODO: Add to GUI
		//this->parentProcessManager->addRobot(this->robotProcessesQFrame);
	}

	ControlledRobot::~ControlledRobot()
	{
	}

	/*void ControlledRobot::handleProcessStat(chrono::system_clock::time_point timeMsgReceived,
	 process_manager::ProcessStat ps)
	 {
	 this->timeLastMsgReceived = timeMsgReceived;
	 auto controlledExecEntry = this->controlledExecMap.find(ps.processKey);
	 if (controlledExecEntry != this->controlledExecMap.end())
	 { // executable is already known

	 // update the statistics of the ControlledExecutable
	 controlledExecEntry->second->handleStat(timeMsgReceived, ps);
	 }
	 else
	 { // executable is unknown
	 cerr << "ControlledRobot: Received processStat for unknown executable with process key " << ps.processKey
	 << endl;
	 return;
	 }
	 }*/

	void ControlledRobot::updateGUI(chrono::system_clock::time_point now)
	{
		/*for (auto controlledExecEntry : this->controlledExecMap)
		 {
		 controlledExecEntry.second->updateGUI(now);
		 }
		 */
	}

} /* namespace rqt_robot_control */
