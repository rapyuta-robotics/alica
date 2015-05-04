#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <rqt_pm_control/PMControl.h>

#include <SystemConfig.h>
#include <RobotExecutableRegistry.h>
#include "rqt_pm_control/ControlledProcessManager.h"
#include "rqt_pm_control/ControlledExecutable.h"
#include "ExecutableMetaData.h"
#include "SigFault.h"

namespace rqt_pm_control
{

	PMControl::PMControl() :
			rqt_gui_cpp::Plugin(), widget_(0)
	{
		setObjectName("PMControl");
		rosNode = new ros::NodeHandle();
		spinner = new ros::AsyncSpinner(4);

		this->sc = supplementary::SystemConfig::getInstance();
		this->msgTimeOut = chrono::duration<double>((*this->sc)["PMControl"]->get<unsigned long>("timeLastMsgReceivedTimeOut", NULL));
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

		// Register executables from Processes.conf
		auto processDescriptions = (*this->sc)["Processes"]->getSections("Processes.ProcessDescriptions", NULL);
		for (auto processSectionName : (*processDescriptions))
		{
			curId = this->pmRegistry->addExecutable(processSectionName);
		}

		// Read bundles from Processes.conf
		auto bundlesSections = (*this->sc)["Processes"]->getSections("Processes.Bundles", NULL);
		for (auto bundleName : (*bundlesSections))
		{
			vector<string> processList = (*this->sc)["Processes"]->getList<string>("Processes.Bundles", bundleName.c_str(), "processList", NULL);
			for (string process : processList)
			{
				this->bundlesMap[bundleName].push_back(stoi(process));
			}
			cout << "PMControl: Bundle '" << bundleName << "' has " << this->bundlesMap[bundleName].size() << " processes." << endl;
		}
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

		// Initialise the ROS Communication
		processStateSub = rosNode->subscribe("/process_manager/ProcessStats", 10, &PMControl::handleProcessStats,
												(PMControl*)this);
		processCommandPub = rosNode->advertise<process_manager::ProcessCommand>("/process_manager/ProcessCommand", 10);
		spinner->start();

		// Initialise the GUI refresh timer
		this->guiUpdateTimer = new QTimer();
		QObject::connect(guiUpdateTimer, SIGNAL(timeout()), this, SLOT(updateGUI()));
		this->guiUpdateTimer->start(1000);

		//This makes segfaults to exceptions
		segfaultdebug::init_segfault_exceptions();
	}

	/**
	 * This method is repeatedly called by the guiUpdateTimer, in order to update the GUI.
	 */
	void PMControl::updateGUI()
	{
		chrono::system_clock::time_point now = chrono::system_clock::now();
		for (auto processManagerEntry : this->processManagersMap)
		{
			if ((now - processManagerEntry.second->timeLastMsgReceived) > this->msgTimeOut)
			{ // time is over, remove process manager

				cout << "PMControl: Erase ControlledProcessManager with ID " << processManagerEntry.second->processManagerId << " from GUI!" << endl;
				this->processManagersMap.erase(processManagerEntry.first);
				delete processManagerEntry.second;
			}
			else
			{ // message arrived before timeout, update its GUI

				processManagerEntry.second->updateGUI(this->ui_.pmHorizontalLayout);
			}
		}
	}

	/**
	 * The callback of the ROS subscriber
	 * @param psts
	 */
	void PMControl::handleProcessStats(process_manager::ProcessStats psts)
	{
		ControlledProcessManager* controlledPM;
		auto pmEntry = this->processManagersMap.find(psts.senderId);
		if (pmEntry != this->processManagersMap.end())
		{
			//cout << "PMControl: ControlledProcessManager with ID " << psts.senderId << " is already known!" << endl;
			controlledPM = pmEntry->second;
		}
		else
		{
			string pmName;
			if (this->pmRegistry->getRobotName(psts.senderId, pmName))
			{
				cout << "PMControl: Create new ControlledProcessManager with ID " << psts.senderId << " and host name " << pmName << "!" << endl;
				controlledPM = new ControlledProcessManager(pmName, msgTimeOut, psts.senderId, this->pmRegistry, this->bundlesMap);
				this->processManagersMap.emplace(psts.senderId, controlledPM);
			}
			else
			{
				cerr << "PMControl: Received message from unknown process manager with sender id " << psts.senderId << endl;
				return;
			}
		}

		controlledPM->handleProcessStats(psts);

	}

	void PMControl::shutdownPlugin()
	{
		this->processCommandPub.shutdown();
		this->processStateSub.shutdown();
	}

	void PMControl::saveSettings(qt_gui_cpp::Settings& plugin_settings, qt_gui_cpp::Settings& instance_settings) const
	{

	}

	void PMControl::restoreSettings(const qt_gui_cpp::Settings& plugin_settings,
									const qt_gui_cpp::Settings& instance_settings)
	{

	}

}

PLUGINLIB_EXPORT_CLASS(rqt_pm_control::PMControl, rqt_gui_cpp::Plugin)
