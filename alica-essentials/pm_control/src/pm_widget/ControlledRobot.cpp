/*
 * ControlledRobot.cpp
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */
#include <ros/ros.h>
#include <limits.h>

#include <process_manager/RobotExecutableRegistry.h>
#include <process_manager/ExecutableMetaData.h>
#include <process_manager/ProcessCommand.h>

#include "pm_widget/ControlledRobot.h"
#include "pm_widget/ControlledExecutable.h"
#include "pm_widget/ControlledProcessManager.h"
#include "ui_RobotProcessesWidget.h"
#include "ui_ProcessWidget.h"

namespace pm_widget
{
	ControlledRobot::ControlledRobot(string robotName, int robotId, ControlledProcessManager* parentProcessManager) :
			RobotMetaData(robotName, robotId), parentProcessManager(parentProcessManager), robotProcessesQFrame(
					new QFrame()), _robotProcessesWidget(new Ui::RobotProcessesWidget())
	{
		// setup gui stuff
		this->_robotProcessesWidget->setupUi(this->robotProcessesQFrame);
		this->_robotProcessesWidget->robotHostLabel->setText(
				QString(string(this->name + " on " + this->parentProcessManager->name).c_str()));

		QObject::connect(this->_robotProcessesWidget->bundleComboBox, SIGNAL(activated(QString)), this,
							SLOT(updateBundles(QString)));

		// enter bundles in combo box
		for (auto bundleEntry : *this->parentProcessManager->bundlesMap)
		{
			this->_robotProcessesWidget->bundleComboBox->insertItem(INT_MAX, QString(bundleEntry.first.c_str()),
																	QVariant(bundleEntry.first.c_str()));
		}

		// construct all known executables
		const vector<supplementary::ExecutableMetaData*>& execMetaDatas =
				this->parentProcessManager->pmRegistry->getExecutables();
		ControlledExecutable* controlledExec;
		for (auto execMetaDataEntry : execMetaDatas)
		{
			controlledExec = new ControlledExecutable(execMetaDataEntry, this);
			this->controlledExecMap.emplace(execMetaDataEntry->id, controlledExec);
		}

		this->parentProcessManager->addRobot(this->robotProcessesQFrame);
	}

	ControlledRobot::~ControlledRobot()
	{
		for (auto execEntry : this->controlledExecMap)
		{
			delete execEntry.second;
		}
		delete robotProcessesQFrame;
	}

	void ControlledRobot::handleProcessStat(chrono::system_clock::time_point timeMsgReceived,
											process_manager::ProcessStat ps)
	{
		auto controlledExecEntry = this->controlledExecMap.find(ps.processKey);
		if (controlledExecEntry != this->controlledExecMap.end())
		{ // executable is already known
			this->timeLastMsgReceived = timeMsgReceived;
			// update the statistics of the ControlledExecutable
			controlledExecEntry->second->handleStat(timeMsgReceived, ps);
		}
		else
		{ // executable is unknown
			cerr << "ControlledRobot: Received processStat for unknown executable with process key " << ps.processKey
					<< endl;
			return;
		}
	}

	void ControlledRobot::updateGUI(chrono::system_clock::time_point now)
	{
		for (auto controlledExecEntry : this->controlledExecMap)
		{
			controlledExecEntry.second->updateGUI(now);
		}
	}

	void ControlledRobot::updateBundles(QString text)
	{
		for (auto controlledExecMapEntry : this->controlledExecMap)
		{
			controlledExecMapEntry.second->handleBundleComboBoxChanged(text);
		}
	}

	void ControlledRobot::addExec(QWidget* exec)
	{
		this->_robotProcessesWidget->verticalLayout->insertWidget(2, exec);
	}

	void ControlledRobot::removeExec(QWidget* exec)
	{
		this->_robotProcessesWidget->verticalLayout->removeWidget(exec);
	}

	void ControlledRobot::sendProcessCommand(vector<int> execIds, vector<int> paramSets, int cmd)
	{
		this->parentProcessManager->sendProcessCommand(vector<int> {this->id}, execIds, paramSets, cmd);
	}
} /* namespace pm_widget */
