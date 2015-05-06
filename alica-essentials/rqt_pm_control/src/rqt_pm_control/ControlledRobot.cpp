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

namespace rqt_pm_control
{
	ControlledRobot::ControlledRobot(string pmName, chrono::duration<double> msgTimeOut, QHBoxLayout* parentHBoxLayout,
										supplementary::RobotExecutableRegistry* pmRegistry, map<string, vector<int>> &bundlesMap, string robotName,
										int robotId, ros::Publisher* processCommandPub) :
			RobotMetaData(robotName, robotId), pmName(pmName), pmRegistry(pmRegistry), msgTimeOut(msgTimeOut), robotProcessesQFrame(nullptr), _robotProcessesWidget(
					nullptr), parentHBoxLayout(parentHBoxLayout), bundlesMap(bundlesMap), initialsed(false), processCommandPub(processCommandPub)
	{
		const vector<supplementary::ExecutableMetaData*>& execMetaDatas = this->pmRegistry->getExecutables();
		ControlledExecutable* controlledExec;
		for (auto execMetaDataEntry : execMetaDatas)
		{
			controlledExec = new ControlledExecutable(execMetaDataEntry->name, execMetaDataEntry->id, execMetaDataEntry->mode,
														execMetaDataEntry->defaultParams, execMetaDataEntry->absExecName);
			this->controlledExecMap.emplace(execMetaDataEntry->id, controlledExec);
		}

	}

	ControlledRobot::~ControlledRobot()
	{
		this->parentHBoxLayout->removeWidget(robotProcessesQFrame);
		delete _robotProcessesWidget;
		delete robotProcessesQFrame;
	}

	void ControlledRobot::init()
	{
		this->robotProcessesQFrame = new QFrame();
		this->_robotProcessesWidget = new Ui::RobotProcessesWidget();
		this->_robotProcessesWidget->setupUi(this->robotProcessesQFrame);
		this->parentHBoxLayout->insertWidget(0, robotProcessesQFrame);
		this->_robotProcessesWidget->robotHostLabel->setText(QString(string(this->name + " on " + this->pmName).c_str()));

		QObject::connect(this->_robotProcessesWidget->bundleComboBox, SIGNAL(activated(QString)), this, SLOT(updateBundles(QString)));

		// enter bundles in combo box
		for (auto bundleEntry : this->bundlesMap)
		{
			this->_robotProcessesWidget->bundleComboBox->insertItem(INT_MAX, QString(bundleEntry.first.c_str()), QVariant(bundleEntry.first.c_str()));
		}

		for (auto controlledExecMapEntry : this->controlledExecMap)
		{
			QObject::connect(controlledExecMapEntry.second, SIGNAL(processCheckBoxStateChanged(int, int)), this, SLOT(handleProcessCheckBoxStateChanged(int,int)), Qt::DirectConnection);
		}

		this->initialsed = true;
	}

	void ControlledRobot::handleProcessStat(process_manager::ProcessStat ps)
	{
		this->timeLastMsgReceived = chrono::system_clock::now();
		//cout << "ControlledRobot: Set last message time to " << this->timeLastMsgReceived.time_since_epoch().count() << endl;

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
		if (!initialsed)
		{
			this->init();
		}

		chrono::system_clock::time_point now = chrono::system_clock::now();

		for (auto controlledExecEntry : this->controlledExecMap)
		{
			// TODO check if bundle selection have been changed and updated the GUI accordingly
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
		auto bundleMapEntry = this->bundlesMap.find(text.toStdString());
		if (bundleMapEntry != this->bundlesMap.end())
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

	void ControlledRobot::handleProcessCheckBoxStateChanged(int newState, int execId)
	{
		process_manager::ProcessCommand pc;
		pc.receiverId = this->id;
		pc.robotIds.push_back(this->id);
		pc.processKeys.push_back(execId);
		switch (newState)
		{
			case Qt::CheckState::Checked:
				pc.cmd = process_manager::ProcessCommand::START;
				break;
			case Qt::CheckState::PartiallyChecked:
				cerr << "ControlledExecutable: What does it mean, that a process is partially checked?!" << endl;
				break;
			case Qt::CheckState::Unchecked:
				pc.cmd = process_manager::ProcessCommand::STOP;
				break;
			default:
				cerr << "ControlledExecutable: Unknown new state of a checkbox!" << endl;
		}

		this->processCommandPub->publish(pc);
	}
} /* namespace rqt_pm_control */
