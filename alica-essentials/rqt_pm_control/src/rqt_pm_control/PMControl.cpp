#include <pluginlib/class_list_macros.h>
#include <ros/master.h>
#include <rqt_pm_control/PMControl.h>

#include <SystemConfig.h>
#include <RobotExecutableRegistry.h>
#include "rqt_pm_control/ControlledProcessManager.h"
#include "rqt_pm_control/ControlledExecutable.h"
#include "ExecutableMetaData.h"

namespace rqt_pm_control
{

	chrono::duration<double> PMControl::msgTimeOut = chrono::duration<double>(0);

	PMControl::PMControl() :
			rqt_gui_cpp::Plugin(), widget_(0), guiUpdateTimer(nullptr)
	{
		setObjectName("PMControl");
		rosNode = new ros::NodeHandle();

		this->sc = supplementary::SystemConfig::getInstance();
		PMControl::msgTimeOut = chrono::duration<double>((*this->sc)["PMControl"]->get<unsigned long>("timeLastMsgReceivedTimeOut", NULL));
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
			vector<string> processParamsList = (*this->sc)["Processes"]->getList<string>("Processes.Bundles", bundleName.c_str(), "processParamsList",
																							NULL);
			if (processList.size() != processParamsList.size())
			{
				cerr << "PMControl: Number of processes does not match the number of parameter sets for the bundle '" << bundleName
						<< "' in the Processes.conf!" << endl;
				continue;
			}

			for (int i = 0; i < processList.size(); i++)
			{
				this->bundlesMap[bundleName].push_back(pair<int, int>(stoi(processList[i]), stoi(processParamsList[i])));
			}
			cout << "PMControl: Bundle '" << bundleName << "' has " << this->bundlesMap[bundleName].size() << " processes." << endl;
		}
	}

	void PMControl::initPlugin(qt_gui_cpp::PluginContext& context)
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
		processStateSub = rosNode->subscribe("/process_manager/ProcessStats", 10, &PMControl::receiveProcessStats, (PMControl*)this);
		processCommandPub = rosNode->advertise<process_manager::ProcessCommand>("/process_manager/ProcessCommand", 10);

		// Initialise the GUI refresh timer
		this->guiUpdateTimer = new QTimer();
		QObject::connect(guiUpdateTimer, SIGNAL(timeout()), this, SLOT(run()));
		this->guiUpdateTimer->start(200);
	}

	/**
	 * The worker method of PMControl. It processes the received ROS messages and afterwards updates the GUI.
	 */
	void PMControl::run()
	{
		handleProcessStats();

		updateGUI();
	}

	/**
	 * Updates the GUI, after ROS process stat message have been processed.
	 */
	void PMControl::updateGUI()
	{
		chrono::system_clock::time_point now = chrono::system_clock::now();
		for (auto processMapIter = this->processManagersMap.begin(); processMapIter != this->processManagersMap.end();)
		{
			if ((now - processMapIter->second->timeLastMsgReceived) > PMControl::msgTimeOut)
			{ // time is over, remove process manager

				cout << "PMControl: The process manager on " << processMapIter->second->name << " (ID: " << processMapIter->second->id
						<< ") seems to be dead!" << endl;
				delete processMapIter->second;
				this->processManagersMap.erase(processMapIter++);
			}
			else
			{ // message arrived before timeout, update its GUI
				processMapIter->second->updateGUI(now);
				++processMapIter;
			}
		}
	}

	void PMControl::addRobot(QFrame* robot)
	{
		this->ui_.pmHorizontalLayout->insertWidget(0, robot);
	}

	void PMControl::removeRobot(QFrame* robot)
	{
		this->ui_.pmHorizontalLayout->removeWidget(robot);
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
			auto timePstsPair = processStatMsgQueue.front();
			processStatMsgQueue.pop();

			// get the corresponding process manager object
			ControlledProcessManager* controlledPM = this->getControlledProcessManager(timePstsPair.second->senderId);
			if (controlledPM != nullptr)
			{
				// hand the message to the process manager, in order to let him update his data structures
				controlledPM->handleProcessStats(timePstsPair);
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
	void PMControl::receiveProcessStats(process_manager::ProcessStatsConstPtr psts)
	{
		lock_guard<mutex> lck(msgQueueMutex);
		this->processStatMsgQueue.emplace(chrono::system_clock::now(), psts);
	}

	void PMControl::sendProcessCommand(int receiverId, vector<int> robotIds, vector<int> execIds, vector<int> paramSets, int cmd)
	{
		process_manager::ProcessCommand pc;
		pc.receiverId = receiverId;
		pc.robotIds = robotIds;
		pc.processKeys = execIds;
		pc.paramSets = paramSets;
		pc.cmd = cmd;
		this->processCommandPub.publish(pc);
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
