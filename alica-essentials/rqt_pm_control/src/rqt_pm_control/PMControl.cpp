#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <rqt_pm_control/PMControl.h>

namespace rqt_pm_control
{

	PMControl::PMControl() :
			rqt_gui_cpp::Plugin(), widget_(0)
	{
		setObjectName("PMControl");
	}

	void PMControl::initPlugin(qt_gui_cpp::PluginContext& context)
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

	bool PMControl::eventFilter(QObject* watched, QEvent* event)
	{
		if (watched == widget_ && event->type() == QEvent::KeyPress)
		{
			QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

			if (keyEvent->key() == Qt::Key_R)
			{
				cout << "R pressed" << endl;
				this->showRBDialog();
				return true;
			}
		}
		return true;
	}

	void PMControl::showRBDialog()
	{
	}

	void PMControl::shutdownPlugin()
	{
	}

	void PMControl::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
	{

	}

	void PMControl::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
	{

	}

}

PLUGINLIB_EXPORT_CLASS(rqt_pm_control::PMControl, rqt_gui_cpp::Plugin)
