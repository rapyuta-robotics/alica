/*
 * ControlledProcessManager.cpp
 *
 *  Created on: Mar 1, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_pm_control/ControlledProcessManager.h"
#include "rqt_pm_control/ControlledRobot.h"
#include <ui_RobotProcessesWidget.h>
#include <RobotExecutableRegistry.h>
#include "rqt_pm_control/PMControl.h"

namespace rqt_pm_control
{
//	QHBoxLayout* parentHBoxLayout, supplementary::RobotExecutableRegistry* pmRegistry,map<string, vector<int>> &bundlesMap, ros::Publisher* processCommandPub, chrono::duration<double> msgTimeOut)
	ControlledProcessManager::ControlledProcessManager(string processManagerName, int processManagerId, PMControl* parentPMControl) :
			name(processManagerName), id(processManagerId), parentPMControl(parentPMControl)
	{

	}

	ControlledProcessManager::~ControlledProcessManager()
	{
		for (auto controlledRobotEntry : this->controlledRobotsMap)
		{
			delete controlledRobotEntry.second;
		}
	}

	void ControlledProcessManager::handleProcessStats(process_manager::ProcessStats psts)
	{
		for (auto processStat : psts.processStats)
		{
			// get the corresponding controlled robot
			ControlledRobot* controlledRobot = this->getControlledRobot(processStat.robotId);
			if (controlledRobot != nullptr)
			{
				// call the controlled robot to update its corresponding process statistics.
				controlledRobot->handleProcessStat(processStat);
			}
		}
	}

	ControlledRobot* ControlledProcessManager::getControlledRobot(int robotId)
	{
		auto controlledRobotEntry = this->controlledRobotsMap.find(robotId);
		if (controlledRobotEntry != this->controlledRobotsMap.end())
		{ // robot is already known
			return controlledRobotEntry->second;
		}
		else
		{ // robot is unknown
			string robotName;
			if (this->parentPMControl->pmRegistry->getRobotName(robotId, robotName))
			{
				cout << "PMControl: Create new ControlledRobot with ID " << robotId << " and robot name " << robotName << "!" << endl;

				ControlledRobot* controlledRobot = new ControlledRobot(robotName, robotId, this);

				this->controlledRobotsMap.emplace(robotId, controlledRobot);
				return controlledRobot;
			}
			else
			{
				cerr << "ControlledRobot: Received processStat from unknown robot with sender id " << robotId << endl;
				return nullptr;
			}
		}
	}

	void ControlledProcessManager::updateGUI()
	{
		chrono::system_clock::time_point now = chrono::system_clock::now();

		for (auto controlledRobotEntry : this->controlledRobotsMap)
		{
			if ((now - controlledRobotEntry.second->timeLastMsgReceived) > this->parentPMControl->msgTimeOut)
			{ // time is over, erase controlled robot

				cout << "ControlledPM: Erasing " << controlledRobotEntry.second->name << ", controlled by process manager " << this->name << " ("
						<< this->id << ") from GUI!" << endl;
				this->controlledRobotsMap.erase(controlledRobotEntry.first);
				delete controlledRobotEntry.second;
			}
			else
			{ // message arrived before timeout, update its GUI

				controlledRobotEntry.second->updateGUI();
			}
		}
	}

	void ControlledProcessManager::sendProcessCommand(vector<int> robotIds, vector<int> execIds, int newState)
	{
		this->parentPMControl->sendProcessCommand(this->id, robotIds, execIds, newState);
	}

} /* namespace rqt_pm_control */

