/*
 * ControlledRobot.cpp
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#include <rqt_pm_control/ControlledRobot.h>
#include <rqt_pm_control/ControlledExecutable.h>
#include <RobotExecutableRegistry.h>
#include <ui_RobotProcessesWidget.h>
#include <limits.h>
#include <process_manager/ProcessCommand.h>
#include <ros/ros.h>
#include "rqt_pm_control/ControlledProcessManager.h"
#include "rqt_pm_control/PMControl.h"

namespace rqt_pm_control
{
	ControlledRobot::ControlledRobot(string robotName, int robotId, ControlledProcessManager* parentProcessManager) :
			RobotMetaData(robotName, robotId), parentProcessManager(parentProcessManager), robotProcessesQFrame(new QFrame()), _robotProcessesWidget(
					new Ui::RobotProcessesWidget())
	{
		// construct all known executables
		const vector<supplementary::ExecutableMetaData*>& execMetaDatas = this->parentProcessManager->parentPMControl->pmRegistry->getExecutables();
		ControlledExecutable* controlledExec;
		for (auto execMetaDataEntry : execMetaDatas)
		{
			controlledExec = new ControlledExecutable(execMetaDataEntry->name, execMetaDataEntry->id, execMetaDataEntry->mode,
														execMetaDataEntry->defaultParams, execMetaDataEntry->absExecName);
			this->controlledExecMap.emplace(execMetaDataEntry->id, controlledExec);
		}

		// setup gui stuff
		this->_robotProcessesWidget->setupUi(this->robotProcessesQFrame);
		this->_robotProcessesWidget->robotHostLabel->setText(QString(string(this->name + " on " + this->parentProcessManager->name).c_str()));

		QObject::connect(this->_robotProcessesWidget->bundleComboBox, SIGNAL(activated(QString)), this, SLOT(updateBundles(QString)));

		// enter bundles in combo box
		for (auto bundleEntry : this->parentProcessManager->parentPMControl->bundlesMap)
		{
			this->_robotProcessesWidget->bundleComboBox->insertItem(INT_MAX, QString(bundleEntry.first.c_str()), QVariant(bundleEntry.first.c_str()));
		}
	}

	ControlledRobot::~ControlledRobot()
	{
		delete _robotProcessesWidget;
		delete robotProcessesQFrame;
		for (auto execEntry : this->controlledExecMap)
		{
			delete execEntry.second;
		}
	}

	void ControlledRobot::handleProcessStat(process_manager::ProcessStat ps)
	{
		this->timeLastMsgReceived = chrono::system_clock::now();

		ControlledExecutable* controlledExec;
		auto controlledExecEntry = this->controlledExecMap.find(ps.processKey);
		if (controlledExecEntry != this->controlledExecMap.end())
		{ // executable is already known
			controlledExec = controlledExecEntry->second;
		}
		else
		{ // executable is unknown
			cerr << "ControlledRobot: Received processStat for unknown executable with process key " << ps.processKey << endl;
			return;
		}

		// update the statistics of the ControlledExecutable
		controlledExec->handleStat(ps);
	}

	void ControlledRobot::updateGUI()
	{
		chrono::system_clock::time_point now = chrono::system_clock::now();

		for (auto controlledExecEntry : this->controlledExecMap)
		{
			controlledExecEntry.second->updateGUI(this->_robotProcessesWidget);
		}
	}

	void ControlledRobot::updateBundles(QString text)
	{
		if (text == "ALL")
		{
			for (auto controlledExecMapEntry : this->controlledExecMap)
			{
				controlledExecMapEntry.second->processWidget->show();
			}
			return;
		}

		if (text == "RUNNING")
		{
			for (auto controlledExecMapEntry : this->controlledExecMap)
			{
				switch (controlledExecMapEntry.second->state)
				{
					case 'R': // running
					case 'S': // interruptable sleeping
					case 'D': // uninterruptable sleeping
					case 'W': // paging
					case 'Z': // zombie
						controlledExecMapEntry.second->processWidget->show();
						break;
					case 'T': // traced, or stopped
					case 'U': // unknown
					default:
						controlledExecMapEntry.second->processWidget->hide();
						break;
				}
			}
			return;
		}

		for (auto controlledExecMapEntry : this->controlledExecMap)
		{
			controlledExecMapEntry.second->processWidget->hide();
		}

		auto bundleMapEntry = this->parentProcessManager->parentPMControl->bundlesMap.find(text.toStdString());
		if (bundleMapEntry != this->parentProcessManager->parentPMControl->bundlesMap.end())
		{
			for (auto processKey : bundleMapEntry->second)
			{
				auto controlledExecMapEntry = this->controlledExecMap.find(processKey);
				if (controlledExecMapEntry != this->controlledExecMap.end())
				{
					controlledExecMapEntry->second->processWidget->show();
				}
			}
		}

	}

	void ControlledRobot::sendProcessCommand(vector<int> execIds, int newState)
	{
		this->parentProcessManager->sendProcessCommand(vector<int> {this->id}, execIds, newState);
	}
} /* namespace rqt_pm_control */
