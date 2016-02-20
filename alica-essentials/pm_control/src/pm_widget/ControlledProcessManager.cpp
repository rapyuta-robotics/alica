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
	ControlledProcessManager::ControlledProcessManager(string processManagerName, int processManagerId, map<string, vector<pair<int, int>>>* bundlesMap, supplementary::RobotExecutableRegistry* pmRegistry, QBoxLayout* parentLayout) :
			name(processManagerName), id(processManagerId), bundlesMap(bundlesMap), pmRegistry(pmRegistry), parentLayout(parentLayout)
	{
		ros::NodeHandle* nh = new ros::NodeHandle();
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		string cmdTopic = (*sc)["ProcessManaging"]->get<string>("Topics.processCmdTopic", NULL);
		processCommandPub = nh->advertise<process_manager::ProcessCommand>(cmdTopic, 10);

		this->msgTimeOut = chrono::duration<double>((*sc)["ProcessManaging"]->get<unsigned long>("PMControl.timeLastMsgReceivedTimeOut", NULL));

	}

	ControlledProcessManager::~ControlledProcessManager()
	{
		for (auto controlledRobotEntry : this->controlledRobotsMap)
		{
			delete controlledRobotEntry.second;
		}
	}

	void ControlledProcessManager::handleProcessStats(
			pair<chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr> timePstsPair)
	{
		this->timeLastMsgReceived = timePstsPair.first;
		for (auto processStat : timePstsPair.second->processStats)
		{
			// get the corresponding controlled robot
			ControlledRobot* controlledRobot = this->getControlledRobot(processStat.robotId);
			if (controlledRobot != nullptr)
			{
				// call the controlled robot to update its corresponding process statistics.
				controlledRobot->handleProcessStat(timePstsPair.first, processStat);
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

				ControlledRobot* controlledRobot = new ControlledRobot(robotName, robotId, this);
				this->controlledRobotsMap.emplace(robotId, controlledRobot);
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
		this->parentLayout->insertWidget(0, robot);
	}

	void ControlledProcessManager::removeRobot(QFrame* robot)
	{
		this->parentLayout->removeWidget(robot);
	}

	void ControlledProcessManager::sendProcessCommand(vector<int> robotIds, vector<int> execIds, vector<int> paramSets,	int cmd)
	{
		process_manager::ProcessCommand pc;
		pc.receiverId = this->id;
		pc.robotIds = robotIds;
		pc.processKeys = execIds;
		pc.paramSets = paramSets;
		pc.cmd = cmd;
		this->processCommandPub.publish(pc);
	}

} /* namespace pm_widget */

