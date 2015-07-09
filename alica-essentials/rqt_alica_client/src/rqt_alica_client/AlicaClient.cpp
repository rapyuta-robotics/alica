#include "../../include/rqt_alica_client/AlicaClient.h"
#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <ros/node_handle.h>
#include <sstream>


namespace rqt_alica_client
{

	AlicaClient::AlicaClient() :
			rqt_gui_cpp::Plugin(), widget_(0)
	{
		setObjectName("AlicaClient");

		rosNode = new ros::NodeHandle();

		aliceClientSubscriber = rosNode->subscribe("/AlicaEngine/BehaviourEngineInfo", 10,
																		&AlicaClient::handleBehaviourEngineInfo,
																		(AlicaClient*)this);

	}

	void AlicaClient::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		widget_ = new QWidget();
		ui_.setupUi(widget_);

		if (context.serialNumber() > 1)
		{
			widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		context.addWidget(widget_);

		widget_->installEventFilter(this);


	}

	void AlicaClient::shutdownPlugin()
	{
	}

	void AlicaClient::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
	{

	}

	void AlicaClient::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
	{

	}

	void AlicaClient::handleBehaviourEngineInfo(
			alica_ros_proxy::AlicaEngineInfoPtr bei) {
		ui_.senderID->setText(QString(bei->senderID));
		ui_.currentPlan->setText(QString(bei->currentPlan.c_str()));
		ui_.currentRole->setText(QString(bei->currentRole.c_str()));
		ui_.currentState->setText(QString(bei->currentState.c_str()));
		ui_.currentTask->setText(QString(bei->currentTask.c_str()));
		ui_.masterPlan->setText(QString(bei->masterPlan.c_str()));
		stringstream ss;
		for(int id : bei->robotIDsWithMe) {
			ss << id << ", ";
		}
		ui_.robotIDsWithMe->setText(QString(ss.str().c_str()));

	}

}

PLUGINLIB_EXPORT_CLASS(rqt_alica_client::AlicaClient, rqt_gui_cpp::Plugin)
