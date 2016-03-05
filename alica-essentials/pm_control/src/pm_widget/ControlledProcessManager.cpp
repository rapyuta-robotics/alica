/*
 * ControlledProcessManager.cpp
 *
 *  Created on: Mar 1, 2015
 *      Author: Stephan Opfer
 */

#include <process_manager/RobotExecutableRegistry.h>
#include <SystemConfig.h>

#include "ui_RobotProcessesWidget.h"
#include "pm_widget/ControlledProcessManager.h"
#include "pm_widget/ControlledRobot.h"

namespace pm_widget
{
	ControlledProcessManager::ControlledProcessManager(string processManagerName, int processManagerId, QBoxLayout* parentLayout) :
			name(processManagerName), id(processManagerId), pmRegistry(supplementary::RobotExecutableRegistry::get()),parentLayout(parentLayout)
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->msgTimeOut = chrono::duration<double>((*sc)["ProcessManaging"]->get<unsigned long>("PMControl.timeLastMsgReceivedTimeOut", NULL));
	}

	ControlledProcessManager::~ControlledProcessManager()
	{
		for (auto& controlledRobotEntry : this->controlledRobotsMap)
		{
			delete controlledRobotEntry.second;
		}
	}

	void ControlledProcessManager::handleProcessStats(
			pair<chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr> timePstsPair)
	{
		this->timeLastMsgReceived = timePstsPair.first;
		for (auto& processStat : timePstsPair.second->processStats)
		{
			// get the corresponding controlled robot
			ControlledRobot* controlledRobot = this->getControlledRobot(processStat.robotId);
			if (controlledRobot != nullptr)
			{
				// call the controlled robot to update its corresponding process statistics.
				controlledRobot->handleProcessStat(timePstsPair.first, processStat, timePstsPair.second->senderId);
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
			if (this->pmRegistry->getRobotName(robotId, robotName))
			{
				cout << "ControlledPM: Create new ControlledRobot " << robotName << " (ID: " << robotId << ")" << endl;

				ControlledRobot* controlledRobot = new ControlledRobot(robotName, robotId, this->id);
				this->controlledRobotsMap.emplace(robotId, controlledRobot);
				this->addRobot(controlledRobot->robotProcessesQFrame);
				return controlledRobot;
			}
			else
			{
				cerr << "ControlledPM: Received processStat from unknown robot with sender id " << robotId << endl;
				return nullptr;
			}
		}
	}

	void ControlledProcessManager::updateGUI(chrono::system_clock::time_point now)
	{
		for (auto robotMapIter = this->controlledRobotsMap.begin(); robotMapIter != this->controlledRobotsMap.end();)
		{
			if ((now - robotMapIter->second->timeLastMsgReceived) > this->msgTimeOut)
			{ // time is over, erase controlled robot
				cout << "ControlledPM: The robot " << robotMapIter->second->name << " (ID: "
						<< robotMapIter->second->id << ") on process manager " << this->name << " (ID: "
						<< this->id << ") seems to be dead!" << endl;
				delete (robotMapIter->second);
				robotMapIter = this->controlledRobotsMap.erase(robotMapIter);
			}
			else
			{ // message arrived before timeout, update its GUI
				robotMapIter->second->updateGUI(now);
				++robotMapIter;
			}
		}
	}

	void ControlledProcessManager::addRobot(QFrame* robot)
	{
		this->parentLayout->addWidget(robot);
	}

	void ControlledProcessManager::removeRobot(QFrame* robot)
	{
		this->parentLayout->removeWidget(robot);
	}

	void ControlledProcessManager::hide()
	{
		for (auto& test : this->controlledRobotsMap)
		{
			test.second->robotProcessesQFrame->hide();
		}
	}

	void ControlledProcessManager::show()
	{
		for (auto& test : this->controlledRobotsMap)
		{
			test.second->robotProcessesQFrame->show();
		}
	}

} /* namespace pm_widget */


