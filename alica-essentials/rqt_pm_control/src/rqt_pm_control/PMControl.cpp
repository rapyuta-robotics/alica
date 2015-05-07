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
			rqt_gui_cpp::Plugin(), widget_(0), guiUpdateTimer(nullptr)
	{
		setObjectName("PMControl");
		rosNode = new ros::NodeHandle();

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
		processStateSub = rosNode->subscribe("/process_manager/ProcessStats", 10, &PMControl::receiveProcessStats, (PMControl*)this);
		processCommandPub = rosNode->advertise<process_manager::ProcessCommand>("/process_manager/ProcessCommand", 10);

		// Initialise the GUI refresh timer
		this->guiUpdateTimer = new QTimer();
		QObject::connect(guiUpdateTimer, SIGNAL(timeout()), this, SLOT(run()));
		this->guiUpdateTimer->start(200);

		//This makes segfaults to exceptions
		segfaultdebug::init_segfault_exceptions();
	}

	/**
	 * Updates the GUI, after ROS process stat message have been processed.
	 */
	void PMControl::updateGUI()
	{
		//ros::spinOnce();
		chrono::system_clock::time_point now = chrono::system_clock::now();
		for (auto processManagerEntry : this->processManagersMap)
		{
			if ((now - processManagerEntry.second->timeLastMsgReceived) > this->msgTimeOut)
			{ // time is over, remove process manager

				cout << "PMControl: Erase ControlledProcessManager with ID " << processManagerEntry.second->id << " from GUI!" << endl;
				this->processManagersMap.erase(processManagerEntry.first);
				delete processManagerEntry.second;
			}
			else
			{ // message arrived before timeout, update its GUI

				processManagerEntry.second->updateGUI();
			}
		}
	}

	/**
	 * Processes all queued ROS process stat messages.
	 */
	void PMControl::handleProcessStats()
	{
		lock_guard<mutex> lck(msgQueueMutex);
		while (!this->processStatMsgQueue.empty())
		{
			// unqueue the ROS process stat message
			process_manager::ProcessStats psts = processStatMsgQueue.front();
			processStatMsgQueue.pop();

			// get the corresponding process manager object
			ControlledProcessManager* controlledPM = this->getControlledProcessManager(psts.senderId);
			if (controlledPM != nullptr)
			{
				// hand the message to the process manager, in order to let him update his data structures
				controlledPM->handleProcessStats(psts);
			}
		}
	}

	/**
	 * If the process manager, corresponding to the given ID, is known, the process manager is returned. If the
	 * ID does not match any known process manager, it searches in the process manager registry for an entry with the
	 * given ID and creates the process manager accordingly. If the registry does not include an entry with the given ID,
	 * an error message is printed and nullptr is returned.
	 * @param processManagerId
	 * @return The ControlledProcessManager object, corresponding to the given ID, or nullptr if nothing is found for the given ID.
	 */
	ControlledProcessManager* PMControl::getControlledProcessManager(int processManagerId)
	{
		auto pmEntry = this->processManagersMap.find(processManagerId);
		if (pmEntry != this->processManagersMap.end())
		{ // process manager is already known
			return pmEntry->second;
		}
		else
		{ // process manager is not known, so create a corresponding instance
			string pmName;
			if (this->pmRegistry->getRobotName(processManagerId, pmName))
			{
				cout << "PMControl: Create new ControlledProcessManager with ID " << processManagerId << " and host name " << pmName << "!" << endl;
				ControlledProcessManager* controlledPM = new ControlledProcessManager(pmName, processManagerId, this);
				//msgTimeOut, this->ui_.pmHorizontalLayout, this->pmRegistry, this->bundlesMap, &this->processCommandPub);
				this->processManagersMap.emplace(processManagerId, controlledPM);
				return controlledPM;
			}
			else
			{
				cerr << "PMControl: Received message from unknown process manager with sender id " << processManagerId << endl;
				return nullptr;
			}
		}
	}

	/**
	 * The callback of the ROS subscriber on ProcessStats messages.
	 * @param psts
	 */
	void PMControl::receiveProcessStats(process_manager::ProcessStats psts)
	{
		lock_guard<mutex> lck(msgQueueMutex);

		this->processStatMsgQueue.push(psts);
	}

	void PMControl::sendProcessCommand(int receiverId, vector<int> robotIds, vector<int> execIds, int newState)
	{
		process_manager::ProcessCommand pc;
		pc.receiverId = receiverId;
		pc.robotIds = robotIds;
		pc.processKeys = execIds;
		switch (newState)
		{
			case Qt::CheckState::Checked:
				pc.cmd = process_manager::ProcessCommand::START;
				break;
			case Qt::CheckState::Unchecked:
				pc.cmd = process_manager::ProcessCommand::STOP;
				break;
			case Qt::CheckState::PartiallyChecked:
				cerr << "PMControl: What does it mean, that a process is partially checked?!" << endl;
				break;
			default:
				cerr << "PMControl: Unknown new state of a checkbox!" << endl;
		}

		this->processCommandPub.publish(pc);
	}

	/**
	 * The worker method of PMControl. It processes the received ROS messages and afterwards updates the GUI.
	 */
	void PMControl::run()
	{
		handleProcessStats();

		updateGUI();
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
