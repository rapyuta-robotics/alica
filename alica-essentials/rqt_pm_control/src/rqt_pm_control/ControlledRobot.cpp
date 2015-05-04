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

namespace rqt_pm_control
{
	ControlledRobot::ControlledRobot(string pmName, chrono::duration<double> msgTimeOut, supplementary::RobotExecutableRegistry* pmRegistry, map<string, vector<int>> &bundlesMap, string robotName, int robotId) :
			RobotMetaData(robotName, robotId), pmName(pmName), pmRegistry(pmRegistry), msgTimeOut(msgTimeOut), robotProcessesQFrame(nullptr), _robotProcessesWidget(nullptr), parentHBoxLayout(nullptr), bundlesMap(bundlesMap)
	{

	}

	ControlledRobot::~ControlledRobot()
	{
		this->parentHBoxLayout->removeWidget(robotProcessesQFrame);
		delete _robotProcessesWidget;
		delete robotProcessesQFrame;
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
			supplementary::ExecutableMetaData const * const execMetaData = this->pmRegistry->getExecutable(ps.processKey);
			if (execMetaData != nullptr)
			{
				cout << "ControlledRobot: Create new ControlledExecutable with ID " << ps.processKey << " and executable name " << execMetaData->name << "!" << endl;
				controlledExec = new ControlledExecutable(execMetaData->name, ps.processKey, execMetaData->mode, execMetaData->defaultParams, execMetaData->absExecName);
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

	void ControlledRobot::updateGUI(QHBoxLayout* parentHBoxLayout)
	{
		chrono::system_clock::time_point now = chrono::system_clock::now();

		if (this->robotProcessesQFrame == nullptr)
		{
			this->parentHBoxLayout = parentHBoxLayout;
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
		}

		for (auto controlledExecEntry : this->controlledExecMap)
		{
			// TODO check if bundle selection have been changed and updated the GUI accordingly
			controlledExecEntry.second->updateGUI(this->_robotProcessesWidget);
		}
	}

	void ControlledRobot::updateBundles(QString text)
	{
		cout << "ControlledRobot: updateBundles called. Param test is '" << text.toStdString() << "'" << endl;
	}

} /* namespace rqt_pm_control */
