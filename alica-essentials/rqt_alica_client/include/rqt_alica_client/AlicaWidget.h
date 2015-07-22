/*
 * AlicaWidget.h
 *
 *  Created on: Jul 22, 2015
 *      Author: Stephan Opfer
 */

#ifndef SRC_RQT_ALICA_CLIENT_ALICAWIDGET_H_
#define SRC_RQT_ALICA_CLIENT_ALICAWIDGET_H_

#include <alica_ros_proxy/AlicaEngineInfo.h>
#include <ui_AlicaClient.h>
namespace rqt_alica
{
	using namespace std;

	class AlicaWidget
	{
	public:
		AlicaWidget();
		virtual ~AlicaWidget();
		void handleAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoPtr bei);

		Ui::AlicaWidget uiAlicaWidget;
		QFrame* qframe;
	};
}
#endif /* SRC_RQT_ALICA_CLIENT_ALICAWIDGET_H_ */
