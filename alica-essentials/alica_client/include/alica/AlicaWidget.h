#pragma once

#include <alica_ros_proxy/AlicaEngineInfo.h>
#include <ui_AlicaWidget.h>
#include <QtGui>
namespace alica
{
	using namespace std;

	class AlicaWidget
	{
	public:
		AlicaWidget();
		virtual ~AlicaWidget();
		void handleAlicaEngineInfo(alica_ros_proxy::AlicaEngineInfoConstPtr aei);
		void clearGUI();

		Ui::AlicaWidget uiAlicaWidget;
		QFrame* qframe;
	};
}
