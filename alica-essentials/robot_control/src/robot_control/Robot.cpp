/*
 * ControlledRobot.cpp
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#include <robot_control/Robot.h>

#include <ros/ros.h>
#include <process_manager/RobotExecutableRegistry.h>

#include "robot_control/RobotCommand.h"
#include "robot_control/RobotsControl.h"
#include "ui_ControlledRobot.h"
#include <alica/AlicaWidget.h>
#include <pm_widget/ControlledProcessManager.h>

#include <chrono>
#include <limits.h>

namespace robot_control
{
	Robot::Robot(string robotName, int robotId, RobotsControl* parentRobotsControl) :
			RobotMetaData(robotName, robotId), parentRobotsControl(parentRobotsControl), widget(new QFrame()), uiControlledRobot(
					new Ui::ControlledRobotWidget()), shown(false), showAlicaClient(false), showProcessManager(false)
	{
		this->uiControlledRobot->setupUi(this->widget);

		// manual configuration of widgets
		this->uiControlledRobot->robotStartStopBtn->setText(QString(this->name.c_str()));

		this->alicaWidget = new alica::AlicaWidget();
		this->uiControlledRobot->scrollAreaWidgetContents->addWidget(this->alicaWidget->qframe);
		this->alicaWidget->qframe->hide();

		this->pmWidget = new pm_widget::ControlledProcessManager(robotName, robotId, &this->parentRobotsControl->bundlesMap ,this->parentRobotsControl->pmRegistry, this->uiControlledRobot->scrollAreaWidgetContents);
		this->pmWidget->hide();


		// signals and slots
		QObject::connect(this->uiControlledRobot->robotStartStopBtn, SIGNAL(toggled(bool)), this,
							SLOT(sendRobotCommand(bool)));
		QObject::connect(this->uiControlledRobot->alicaShowHideBtn, SIGNAL(toggled(bool)), this,
							SLOT(toggleAlicaClient(bool)));
		QObject::connect(this->uiControlledRobot->pmShowHideBtn, SIGNAL(toggled(bool)), this,
							SLOT(toggleProcessManager(bool)));

		// hide by default
		this->widget->hide();
		this->uiControlledRobot->scrollArea->hide();

		// add to parent widget
		this->parentRobotsControl->robotControlWidget_.allRobotsFlowLayout->addWidget(this->widget);

		this->robotCommandPub = this->parentRobotsControl->rosNode->advertise<robot_control::RobotCommand>("RobotCommand",5);
	}

	Robot::~Robot()
	{
	}

	void Robot::toggleAlicaClient(bool show)
	{
		this->showAlicaClient = show;

		if (showAlicaClient)
		{
			this->uiControlledRobot->scrollArea->show();
			this->alicaWidget->qframe->show();
		}
		else
		{
			this->alicaWidget->qframe->hide();
			if (!showProcessManager)
			{
				this->uiControlledRobot->scrollArea->hide();
			}
		}

	}

	void Robot::toggleProcessManager(bool show)
	{
		this->showProcessManager = show;

		if (showProcessManager)
		{
			this->uiControlledRobot->scrollArea->show();
			this->pmWidget->show();
		}
		else
		{
			this->pmWidget->hide();
			if (!showAlicaClient)
			{
				this->uiControlledRobot->scrollArea->hide();
			}
		}

	}

	void Robot::updateGUI(chrono::system_clock::time_point now)
	{
		if (chrono::system_clock::now() - this->timeLastMsgReceived > std::chrono::milliseconds(1000))
		{
			this->clearGUI();
		}
		else
		{
			this->pmWidget->updateGUI(now);
		}
	}

	void Robot::clearGUI()
	{
		this->alicaWidget->clearGUI();
		// TODO: clear process manager
	}

	void Robot::sendRobotCommand(bool start)
	{
		RobotCommand rc;
		rc.receiverId = this->id;
		if (start)
		{
			rc.cmd = RobotCommand::START;
		}
		else
		{
			rc.cmd = RobotCommand::STOP;
		}
		this->robotCommandPub.publish(rc);
	}

	void Robot::toggle()
	{
		if (shown)
		{
			this->hide();
		}
		else
		{
			this->show();
		}
	}

	void Robot::show()
	{
		this->shown = true;
		this->uiControlledRobot->scrollArea->adjustSize();
		this->widget->show();
	}

	void Robot::hide()
	{
		this->shown = false;
		this->uiControlledRobot->scrollArea->adjustSize();
		this->widget->hide();
	}

	void Robot::handleAlicaInfo(pair<chrono::system_clock::time_point, alica_ros_proxy::AlicaEngineInfoConstPtr> timeAEIpair)
	{
		this->timeLastMsgReceived = timeAEIpair.first;
		this->alicaWidget->handleAlicaEngineInfo(timeAEIpair.second);
	}

	void Robot::handleProcessStats(pair<chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr> timePSTSpair)
	{
		this->timeLastMsgReceived = timePSTSpair.first;
		this->pmWidget->handleProcessStats(timePSTSpair);
	}


} /* namespace robot_control */
