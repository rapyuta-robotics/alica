#ifndef rqt_alica_client__AlicaClient_H
#define rqt_alica_client__AlicaClient_H

#include <rqt_gui_cpp/plugin.h>
#include <alica_ros_proxy/AlicaEngineInfo.h>

#include <ui_AlicaClient.h>

#include <ros/macros.h>
#include <ros/subscriber.h>
#include <QtGui>
#include <QWidget>
#include <QDialog>

namespace rqt_alica_client
{
	using namespace std;


	class AlicaClient : public rqt_gui_cpp::Plugin
	{

	Q_OBJECT

	public:

		AlicaClient();

		virtual void initPlugin(qt_gui_cpp::PluginContext& context);

		virtual void shutdownPlugin();

		virtual void saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const;

		virtual void restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings);

		void handleAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoPtr bei);

		Ui::AlicaClientWidget ui_;

		QWidget* widget_;

	private:

		ros::NodeHandle* rosNode;
		ros::Subscriber aliceClientSubscriber;

	};

}

#endif // rqt_alica_client__AlicaClient_H
