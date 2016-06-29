/*
 * AlicaWidget.h
 *
 *  Created on: Jul 22, 2015
 *      Author: Stephan Opfer
 */

#ifndef SRC_RQT_ALICA_CLIENT_ALICAWIDGET_H_
#define SRC_RQT_ALICA_CLIENT_ALICAWIDGET_H_

#include <alica_ros_proxy/AlicaEngineInfo.h>
#include <msl_actuator_msgs/KickerStatInfo.h>
#include <ui_AlicaWidget.h>
namespace alica
{
	using namespace std;

	class AlicaWidget
	{
	public:
		AlicaWidget();
		virtual ~AlicaWidget();
		void handleAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoConstPtr aei);
		void handleKickerStatInfo(msl_actuator_msgs::KickerStatInfoPtr kickStatInfo);
		void clearGUI();

		Ui::AlicaWidget uiAlicaWidget;
		QFrame* qframe;
	};
}
#endif /* SRC_RQT_ALICA_CLIENT_ALICAWIDGET_H_ */
