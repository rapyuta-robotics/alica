/*
 * ControlledProcessManager.cpp
 *
 *  Created on: Mar 1, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_pm_control/ControlledProcessManager.h"
#include <ui_RobotProcessesWidget.h>

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

	void ControlledProcessManager::ProcessMessage(process_manager::ProcessStats psts)
	{

		this->lastTimeMsgReceived = chrono::system_clock::now();
		cout << "CPMan: Set last message time to " << this->lastTimeMsgReceived.time_since_epoch().count() << endl;

		// TODO: create robots and its processes
	}

	void ControlledProcessManager::updateGUI(QHBoxLayout* parentLayout)
	{
		if (this->_processManagerWidget == nullptr)
		{
			this->_processManagerWidget = new Ui::RobotProcessesWidget();
			this->robotProc = new QFrame();
			this->_processManagerWidget->setupUi(this->robotProc);
		}

		// TODO: This is just for testing! There Should be one RobotProcessesWidget per ControlledRobot!!!
		this->_processManagerWidget->robotHostLabel->setText(QString(this->name.c_str()));
		if (robotProc->parent() == nullptr)
		{
			parentLayout->insertWidget(0, robotProc);
		}
	}

} /* namespace rqt_pm_control */

