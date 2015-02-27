#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <rqt_pm_control/PMControl.h>

namespace rqt_pm_control
{

	PMControl::PMControl() :
			rqt_gui_cpp::Plugin(), widget_(0)
	{
		setObjectName("PMControl");
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);
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

		processStateSub = rosNode->subscribe("/process_manager/ProcessStats", 10, &PMControl::handleProcessStats, (PMControl*)this);
		processCommandPub = rosNode->advertise<process_manager::ProcessCommand>("/process_manager/ProcessCommand", 10);
		spinner->start();
	}

	/**
	 * The callback of the ROS subscriber - it inits the message processing.
	 * @param pc
	 */
	void PMControl::handleProcessStats(process_manager::ProcessStats psts)
	{
		cout << "PMControl: Received " << psts.processStats.size() << "Process Stats! " << endl;
		auto procManEntry = this->procMan2RobotsMap.find(psts.senderId);
		if (procManEntry == this->procMan2RobotsMap.end())
		{
			this->procMan2RobotsMap.emplace(psts.senderId, vector<ObservedRobot*>());
		}

		for (auto procStat : psts.processStats)
		{
			this->procMan2RobotsMap[psts.senderId].push_back(new ObservedRobot("TestName", procStat.robotId));
		}

	}

	bool PMControl::eventFilter(QObject* watched, QEvent* event)
	{
		if (watched == widget_ && event->type() == QEvent::KeyPress)
		{
			QKeyEvent *keyEvent = static_cast<QKeyEvent *>(event);

			if (keyEvent->key() == Qt::Key_R)
			{
				cout << "R pressed" << endl;
				return true;
			}
		}
		return true;
	}

	void PMControl::shutdownPlugin()
	{
		this->processCommandPub.shutdown();
		this->processStateSub.shutdown();
	}

	void PMControl::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
	{

	}

	void PMControl::restoreSettings(const qt_gui_cpp::Settings& plugin_settings, const qt_gui_cpp::Settings& instance_settings)
	{

	}

}

PLUGINLIB_EXPORT_CLASS(rqt_pm_control::PMControl, rqt_gui_cpp::Plugin)
