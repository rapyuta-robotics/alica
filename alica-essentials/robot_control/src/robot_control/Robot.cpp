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
//#include <pm_widget/ControlledProcessManager.h>
#include <pm_widget/ControlledRobot.h>
#include <QFrame>

#include <chrono>
#include <limits.h>

namespace robot_control
{
	Robot::Robot(string robotName, int robotId, RobotsControl* parentRobotsControl) :
			RobotMetaData(robotName, robotId), parentRobotsControl(parentRobotsControl), widget(new QFrame()), uiControlledRobot(
					new Ui::ControlledRobotWidget()), shown(true), showAlicaClient(true), showProcessManager(true)
	{
		this->uiControlledRobot->setupUi(this);

		// manual configuration of widgets
		this->uiControlledRobot->robotStartStopBtn->setText(QString(std::string(this->name + " (" + std::to_string(robotId) + ")" ).c_str()));

		frameForAW = new QFrame(this);
		frameForAW->setFrameStyle(1);
		frameForAW->setMaximumSize(280,500);
		frameForAW->setContentsMargins(0,0,0,0);
		QHBoxLayout* frameHBoxAw = new QHBoxLayout(frameForAW);
		frameHBoxAw->setContentsMargins(0,0,0,0);
		frameHBoxAw->setAlignment(Qt::AlignmentFlag::AlignTop);
		this->alicaWidget = new alica::AlicaWidget();
		frameHBoxAw->addWidget(this->alicaWidget->qframe);
		this->uiControlledRobot->horizontalLayout_2->addWidget(frameForAW);

		frameForPM = new QFrame(this);
		frameForPM->setFrameStyle(1);
		frameForPM->setMaximumSize(280,500);
		frameForPM->setContentsMargins(0,0,0,0);
		QHBoxLayout* frameHBoxPm = new QHBoxLayout(frameForPM);
		frameHBoxPm->setContentsMargins(0,0,0,0);
		frameHBoxPm->setAlignment(Qt::AlignmentFlag::AlignTop);
		this->controlledRobotWidget = new pm_widget::ControlledRobot(robotName, robotId, -1); // -1 means, there is no process manager responsible
		frameHBoxPm->addWidget(this->controlledRobotWidget->robotProcessesQFrame);
		this->uiControlledRobot->horizontalLayout_2->addWidget(frameForPM);

		// signals and slots
		QObject::connect(this->uiControlledRobot->robotStartStopBtn, SIGNAL(toggled(bool)), this,
							SLOT(sendRobotCommand(bool)));
		QObject::connect(this->uiControlledRobot->alicaShowHideBtn, SIGNAL(toggled(bool)), this,
							SLOT(toggleAlicaClient(bool)));
		QObject::connect(this->uiControlledRobot->pmShowHideBtn, SIGNAL(toggled(bool)), this,
							SLOT(toggleProcessManager(bool)));

		//vBox->addWidget(new QSpacerItem(0,0,QSizePolicy::Expanding, QSizePolicy::Line));
		this->parentRobotsControl->robotControlWidget_.robotControlLayout->addWidget(this);

		this->robotCommandPub = this->parentRobotsControl->rosNode->advertise<robot_control::RobotCommand>("RobotCommand",5);
	}

	Robot::~Robot()
	{
	}

	QSize Robot::sizeHint()
	{
		return QSize(500,500);
	}

	void Robot::toggleAlicaClient(bool show)
	{
		this->showAlicaClient = show;

		if (showAlicaClient)
		{
			this->frameForAW->show();
			this->uiControlledRobot->scrollArea->show();
		}
		else
		{
			this->frameForAW->hide();
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
			this->frameForPM->show();
			this->uiControlledRobot->scrollArea->show();
		}
		else
		{
			this->frameForPM->hide();
			if (!showAlicaClient)
			{
				this->uiControlledRobot->scrollArea->hide();
			}
		}

	}

	void Robot::updateGUI(chrono::system_clock::time_point now)
	{
		if ((now - this->timeLastMsgReceived) > std::chrono::milliseconds(1000))
		{
			this->alicaWidget->clearGUI();
		}
		this->controlledRobotWidget->updateGUI(now);
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
		QFrame::show();
	}

	void Robot::hide()
	{
		this->shown = false;
		this->uiControlledRobot->scrollArea->adjustSize();
		QFrame::hide();
	}

	void Robot::handleAlicaInfo(pair<chrono::system_clock::time_point, alica_ros_proxy::AlicaEngineInfoConstPtr> timeAEIpair)
	{
		this->timeLastMsgReceived = timeAEIpair.first;
		this->alicaWidget->handleAlicaEngineInfo(timeAEIpair.second);
	}

	/*void Robot::handleKickerStatInfo(pair<chrono::system_clock::time_point, msl_actuator_msgs::KickerStatInfoPtr> timeKSIpair)
	{
		this->timeLastMsgReceived = timeKSIpair.first;
		this->alicaWidget->handleKickerStatInfo(timeKSIpair.second);
	}*/

	void Robot::handleProcessStat(chrono::system_clock::time_point timeMsgReceived, process_manager::ProcessStat ps, int parentPMid)
	{
		this->timeLastMsgReceived = timeMsgReceived;
		this->controlledRobotWidget->handleProcessStat(timeMsgReceived, ps, parentPMid);
	}


} /* namespace robot_control */
