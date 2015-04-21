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

	ControlledProcessManager::ControlledProcessManager(string name, int processManagerId) :
			name(name), processManagerId(processManagerId)
	{

	}

	ControlledProcessManager::~ControlledProcessManager()
	{
		cout << "CPMan: Destructor called!" << endl;
		delete this->robotProc;
		delete this->_processManagerWidget;
	}

	void ControlledProcessManager::handleProcessStats(process_manager::ProcessStats psts, supplementary::RobotExecutableRegistry* pmRegistry)
	{
		this->timeLastMsgReceived = chrono::system_clock::now();
		cout << "CPMan: Set last message time to " << this->timeLastMsgReceived.time_since_epoch().count() << endl;

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
				if (pmRegistry->getRobotName(processStat.robotId, robotName))
				{
					cout << "PMControl: Create new ControlledRobot with ID " << processStat.robotId << " and robot name " << robotName << "!" << endl;
					controlledRobot = new ControlledRobot(robotName, processStat.robotId);
					this->controlledRobotsMap.emplace(processStat.robotId, controlledRobot);
				}
				else
				{
					cerr << "ControlledPM: Received processStat from unknown robot with sender id " << processStat.robotId << endl;
					return;
				}
			}

			// call the controlled robot to update its corresponding process statistics.
			controlledRobot->handleProcessStat(processStat, pmRegistry);
		}
	}

	void ControlledProcessManager::updateGUI(QHBoxLayout* parentLayout)
	{
		if (this->_processManagerWidget == nullptr)
		{
			this->_processManagerWidget = new Ui::RobotProcessesWidget();
			this->robotProc = new QFrame();
			this->_processManagerWidget->setupUi(this->robotProc);
		}

		// TODO: This is just for testing! There should be one RobotProcessesWidget per ControlledRobot!!!
		this->_processManagerWidget->robotHostLabel->setText(QString(this->name.c_str()));
		if (robotProc->parent() == nullptr)
		{
			parentLayout->insertWidget(0, robotProc);
		}
	}

} /* namespace rqt_pm_control */

