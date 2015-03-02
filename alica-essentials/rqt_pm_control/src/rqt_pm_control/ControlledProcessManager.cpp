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

	ControlledProcessManager::ControlledProcessManager(string name, int processManagerId) : name(name), processManagerId(processManagerId)
	{
	}

	ControlledProcessManager::~ControlledProcessManager()
	{
		// TODO: hang out the widget and destruct the map entry#
	}

	void ControlledProcessManager::ProcessMessage(process_manager::ProcessStats psts)
	{
		this->lastTimeMsgReceived = chrono::system_clock::now();

		// TODO: create robots and its processes
	}

	void ControlledProcessManager::updateGUI(QHBoxLayout* parentLayout)
	{
		if (this->_processManagerWidget == nullptr)
		{
			this->_processManagerWidget = new Ui::RobotProcessesWidget();
			this->_processManagerWidget->setupUi(this->robotProc);
		}

		// TODO: This is just for testing! There Should be one RobotProcessesWidget per ControlledRobot!!!
		this->_processManagerWidget->robotHostLabel->setText(QString(this->name.c_str()));
		parentLayout->addWidget(robotProc);

	}

} /* namespace rqt_pm_control */


