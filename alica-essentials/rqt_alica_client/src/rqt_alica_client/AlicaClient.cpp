#include "../../include/rqt_alica_client/AlicaClient.h"
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/node_handle.h>
#include <sstream>

namespace rqt_alica_client
{

	AlicaClient::AlicaClient() :
			rqt_gui_cpp::Plugin(), widget(0)
	{
		setObjectName("AlicaClient");

		rosNode = new ros::NodeHandle();

		aliceClientSubscriber = rosNode->subscribe("/AlicaEngine/AlicaEngineInfo", 10,
													&AlicaClient::handleAlicaEngineInfo, (AlicaClient*)this);

	}

	void AlicaClient::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		widget = new QWidget();
		uiAlicaClientWidget.setupUi(widget);

		if (context.serialNumber() > 1)
		{
			widget->setWindowTitle(widget->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		context.addWidget(widget);
	}

	void AlicaClient::shutdownPlugin()
	{
	}

	void AlicaClient::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
	{

	}

	void AlicaClient::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
										const qt_gui_cpp::Settings& instance_settings)
	{

	}

	void AlicaClient::handleAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoPtr bei)
	{
		uiAlicaClientWidget.planVal->setText(QString(bei->currentPlan.c_str()));
		uiAlicaClientWidget.roleVal->setText(QString(bei->currentRole.c_str()));
		uiAlicaClientWidget.taskVal->setText(QString(bei->currentTask.c_str()));
		uiAlicaClientWidget.masterPlanVal->setText(QString(bei->masterPlan.c_str()));

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

		uiAlicaClientWidget.stateVal->setText(QString(ss.str().c_str()));
	}

}

PLUGINLIB_EXPORT_CLASS(rqt_alica_client::AlicaClient, rqt_gui_cpp::Plugin)
