/*
 * ControlledRobot.cpp
 *
 *  Created on: Feb 27, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_robot_control/ControlledRobot.h"
#include "rqt_robot_control/RobotsControl.h"
#include "ui_ControlledRobot.h"

#include <RobotExecutableRegistry.h>
#include <limits.h>
#include <ros/ros.h>

namespace rqt_robot_control
{
	ControlledRobot::ControlledRobot(string robotName, int robotId, RobotsControl* parentRobotsControl) :
			RobotMetaData(robotName, robotId), parentRobotsControl(parentRobotsControl), widget(new QFrame()), uiControlledRobot(
					new Ui::ControlledRobotWidget()), shown(false), showAlicaClient(false), showProcessManager(false)
	{
		this->uiControlledRobot->setupUi(this->widget);
		this->uiControlledRobot->robotStartStopBtn->setText(QString(this->name.c_str()));
		QObject::connect(this->uiControlledRobot->robotStartStopBtn, SIGNAL(toggled(bool)), this,
							SLOT(sendRobotCommand(bool)));
		QObject::connect(this->uiControlledRobot->alicaShowHideBtn, SIGNAL(toggled(bool)), this,
							SLOT(toggleAlicaClient(bool)));
		QObject::connect(this->uiControlledRobot->pmShowHideBtn, SIGNAL(toggled(bool)), this,
							SLOT(toggleProcessManager(bool)));
		this->widget->hide();
		this->uiControlledRobot->scrollArea->hide();
		this->parentRobotsControl->robotControlWidget_.allRobotsFlowLayout->addWidget(this->widget);
	}

	ControlledRobot::~ControlledRobot()
	{
	}

	void ControlledRobot::toggleAlicaClient(bool show)
	{
		this->showAlicaClient = show;

		if (showAlicaClient)
		{
			this->uiControlledRobot->scrollArea->show();
		}
		else if (!showAlicaClient && !showProcessManager)
		{
			this->uiControlledRobot->scrollArea->hide();
		}
		this->uiControlledRobot->scrollArea->adjustSize();
	}

	void ControlledRobot::toggleProcessManager(bool show)
	{
		this->showProcessManager = show;

		if (showProcessManager)
		{
			this->uiControlledRobot->scrollArea->show();
		}
		else if (!showAlicaClient && !showProcessManager)
		{
			this->uiControlledRobot->scrollArea->hide();
		}

		this->uiControlledRobot->scrollArea->adjustSize();
	}

	void ControlledRobot::updateGUI(chrono::system_clock::time_point now)
	{
	}

	void ControlledRobot::sendRobotCommand(bool start)
	{
		if (start)
		{
			cout << "CR: sendStart()" << endl;
		}
		else
		{
			cout << "CR: sendStop()" << endl;
		}
	}

	void ControlledRobot::toggle()
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

	void ControlledRobot::show()
	{
		this->shown = true;
		this->uiControlledRobot->scrollArea->adjustSize();
		this->widget->show();
	}

	void ControlledRobot::hide()
	{
		this->shown = false;
		this->uiControlledRobot->scrollArea->adjustSize();
		this->widget->hide();
	}

} /* namespace rqt_robot_control */
