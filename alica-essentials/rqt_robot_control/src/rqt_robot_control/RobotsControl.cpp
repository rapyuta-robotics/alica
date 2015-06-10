#include <pluginlib/class_list_macros.h>
#include <ros/master.h>

#include <rqt_robot_control/RobotsControl.h>

#include <SystemConfig.h>
#include <RobotExecutableRegistry.h>

namespace rqt_robot_control
{

	chrono::duration<double> RobotsControl::msgTimeOut = chrono::duration<double>(0);

	RobotsControl::RobotsControl() :
			rqt_gui_cpp::Plugin(), widget_(0), guiUpdateTimer(nullptr)
	{
		setObjectName("RobotsControl");
		rosNode = new ros::NodeHandle();

		this->sc = supplementary::SystemConfig::getInstance();
		RobotsControl::msgTimeOut = chrono::duration<double>(
				(*this->sc)["PMControl"]->get<unsigned long>("timeLastMsgReceivedTimeOut", NULL));
		this->pmRegistry = new supplementary::RobotExecutableRegistry();

		/* Initialise the registry data structure for better performance
		 * with data from Globals.conf and Processes.conf file. */

		// Register robots from Globals.conf
		int curId;
		auto robotNames = (*this->sc)["Globals"]->getSections("Globals.Team", NULL);
		for (auto robotName : (*robotNames))
		{
			curId = this->pmRegistry->addRobot(robotName);
		}
	}

	void RobotsControl::initPlugin(qt_gui_cpp::PluginContext& context)
	{
		widget_ = new QWidget();
		widget_->setAttribute(Qt::WA_AlwaysShowToolTips, true);
		ui_.setupUi(widget_);

		if (context.serialNumber() > 1)
		{
			widget_->setWindowTitle(widget_->windowTitle() + " (" + QString::number(context.serialNumber()) + ")");
		}
		context.addWidget(widget_);

		// Initialise the ROS Communication
		processStateSub = rosNode->subscribe("/process_manager/ProcessStats", 10, &RobotsControl::receiveProcessStats,
												(RobotsControl*)this);
		alicaInfoSub = rosNode->subscribe("/alica_engine/AlicaEngineInfo", 10, &RobotsControl::receiveAlicaInfo,
											(RobotsControl*)this);

		// Initialise the GUI refresh timer
		this->guiUpdateTimer = new QTimer();
		QObject::connect(guiUpdateTimer, SIGNAL(timeout()), this, SLOT(run()));
		this->guiUpdateTimer->start(200);
	}

	/**
	 * The worker method of RobotsControl. It processes the received ROS messages and afterwards updates the GUI.
	 */
	void RobotsControl::run()
	{
		processMessages();

		updateGUI();
	}

	/**
	 * Updates the GUI, after ROS process stat message have been processed.
	 */
	void RobotsControl::updateGUI()
	{
		chrono::system_clock::time_point now = chrono::system_clock::now();
		// TODO: Not implemented, yet.
	}

	void RobotsControl::receiveProcessStats(process_manager::ProcessStatsConstPtr processStats)
	{
		lock_guard<mutex> lck(processStatsMsgQueueMutex);
		this->processStatMsgQueue.emplace(chrono::system_clock::now(), processStats);
	}

	void RobotsControl::receiveAlicaInfo(alica_ros_proxy::AlicaEngineInfoConstPtr alicaInfo)
	{
		lock_guard<mutex> lck(alicaInfoMsgQueueMutex);
		this->alicaInfoMsgQueue.emplace(chrono::system_clock::now(), alicaInfo);
	}

	void RobotsControl::processMessages()
	{
		{
			lock_guard<mutex> lck(processStatsMsgQueueMutex);
			while (!this->processStatMsgQueue.empty())
			{
				// unqueue the ROS process stat message
				auto timePstsPair = processStatMsgQueue.front();
				processStatMsgQueue.pop();

				this->checkAndInit(timePstsPair.second->senderId);
			}
		}

		{
			lock_guard<mutex> lck(alicaInfoMsgQueueMutex);
			while (!this->alicaInfoMsgQueue.empty())
			{
				// unqueue the ROS process stat message
				auto timePstsPair = alicaInfoMsgQueue.front();
				alicaInfoMsgQueue.pop();

				this->checkAndInit(timePstsPair.second->senderID);
			}
		}

	}

	/**
	 * If the given robot ID is already known, nothing is done.
	 * Otherwise a new entry in the controlled robot map is created.
	 */
	void RobotsControl::checkAndInit(int robotId)
	{
		auto pmEntry = this->controlledRobotsMap.find(robotId);
		if (pmEntry == this->controlledRobotsMap.end())
		{ // robot is not known, so create a corresponding instance
			string robotName;
			if (this->pmRegistry->getRobotName(robotId, robotName))
			{
				cout << "RobotsControl: Create new ControlledRobot with ID " << robotId << " and host name "
						<< robotName << "!" << endl;
				ControlledRobot* controlledRobot = new ControlledRobot(robotName, robotId, this);
				this->controlledRobotsMap.emplace(robotId, controlledRobot);
			}
			else
			{
				cerr << "RobotsControl: Received message from unknown robot with sender id " << robotId << endl;
			}
		}
	}

	void RobotsControl::shutdownPlugin()
	{
		this->processStateSub.shutdown();
		this->alicaInfoSub.shutdown();
	}

	void RobotsControl::saveSettings(qt_gui_cpp::Settings& plugin_settings,
										qt_gui_cpp::Settings& instance_settings) const
	{

	}

	void RobotsControl::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
										const qt_gui_cpp::Settings& instance_settings)
	{

	}

}

PLUGINLIB_EXPORT_CLASS(rqt_robot_control::RobotsControl, rqt_gui_cpp::Plugin)
