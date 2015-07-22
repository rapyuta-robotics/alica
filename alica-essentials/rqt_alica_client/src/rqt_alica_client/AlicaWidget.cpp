/*
 * AlicaWidget.cpp
 *
 *  Created on: Jul 22, 2015
 *      Author: Stephan Opfer
 */

#include "rqt_alica_client/AlicaWidget.h"
#include <sstream>

namespace rqt_alica
{
	AlicaWidget::AlicaWidget() : qframe(new QFrame())
	{
		this->uiAlicaWidget.setupUi(qframe);
	}

	AlicaWidget::~AlicaWidget()
	{
		delete qframe;
	}

	void AlicaWidget::handleAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoPtr bei)
	{
		uiAlicaWidget.planVal->setText(QString(bei->currentPlan.c_str()));
		uiAlicaWidget.roleVal->setText(QString(bei->currentRole.c_str()));
		uiAlicaWidget.taskVal->setText(QString(bei->currentTask.c_str()));
		uiAlicaWidget.masterPlanVal->setText(QString(bei->masterPlan.c_str()));

		stringstream ss;
		ss << bei->currentState << " (";
		if (bei->robotIDsWithMe.size() > 0)
		{
			for (int i = 0; i < bei->robotIDsWithMe.size() - 1; i++)
			{
				ss << bei->robotIDsWithMe[i] << ", ";
			}
			ss << bei->robotIDsWithMe[bei->robotIDsWithMe.size()];
		}
		ss << ")";

		uiAlicaWidget.stateVal->setText(QString(ss.str().c_str()));
	}
}
