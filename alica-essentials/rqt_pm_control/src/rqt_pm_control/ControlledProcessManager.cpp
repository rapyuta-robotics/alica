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

namespace rqt_pm_control
{

	ControlledProcessManager::ControlledProcessManager(string name, chrono::duration<double> msgTimeOut, int processManagerId, supplementary::RobotExecutableRegistry* pmRegistry, map<string, vector<int>> &bundlesMap) :
			name(name), processManagerId(processManagerId), msgTimeOut(msgTimeOut), pmRegistry(pmRegistry), parentHBoxLayout(parentHBoxLayout), bundlesMap(bundlesMap)
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
		this->timeLastMsgReceived = chrono::system_clock::now();
		//cout << "CPMan: Set last message time to " << this->timeLastMsgReceived.time_since_epoch().count() << endl;

		ControlledRobot* controlledRobot;
		for (auto processStat : psts.processStats)
		{
			auto controlledRobotEntry = this->controlledRobotsMap.find(processStat.robotId);
			if (controlledRobotEntry != this->controlledRobotsMap.end())
			{ // robot is already known
				controlledRobot = controlledRobotEntry->second;
			}
			else
			{ // robot is unknown
				string robotName;
				if (this->pmRegistry->getRobotName(processStat.robotId, robotName))
				{
					cout << "PMControl: Create new ControlledRobot with ID " << processStat.robotId << " and robot name " << robotName << "!" << endl;
					controlledRobot = new ControlledRobot(this->name,  this->msgTimeOut, this->pmRegistry, this->bundlesMap, robotName, processStat.robotId);
					this->controlledRobotsMap.emplace(processStat.robotId, controlledRobot);
				}
				else
				{
					cerr << "ControlledPM: Received processStat from unknown robot with sender id " << processStat.robotId << endl;
					return;
				}
			}

			// call the controlled robot to update its corresponding process statistics.
			controlledRobot->handleProcessStat(processStat);
		}
	}

	void ControlledProcessManager::updateGUI(QHBoxLayout* parentHBoxLayout)
	{
		chrono::system_clock::time_point now = chrono::system_clock::now();
		if (this->parentHBoxLayout != nullptr)
		{
			this->parentHBoxLayout = parentHBoxLayout;
		}

		for (auto controlledRobotEntry : this->controlledRobotsMap)
		{
			if ((now - controlledRobotEntry.second->timeLastMsgReceived) > this->msgTimeOut)
			{ // time is over, erase controlled robot

				cout << "ControlledPM: Erasing " << controlledRobotEntry.second->name << ", controlled by process manager " << this->name << " (" << this->processManagerId << ") from GUI!" << endl;
				this->controlledRobotsMap.erase(controlledRobotEntry.first);
				delete controlledRobotEntry.second;
			}
			else
			{ // message arrived before timeout, update its GUI

				controlledRobotEntry.second->updateGUI(parentHBoxLayout);
			}
		}
	}

} /* namespace rqt_pm_control */

