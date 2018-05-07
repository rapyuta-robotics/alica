#include <process_manager/ProcessStats.h>
#include <process_manager/ProcessCommand.h>

#include <ros/ros.h>
#include <QFrame>
#include <QBoxLayout>
#include <string>
#include <utility>
#include <chrono>

#include <supplementary/AgentID.h>

namespace supplementary {
	class SystemConfig;
	class RobotExecutableRegistry;
}

namespace pm_control {
	class PMControl;
}

namespace pm_widget
{
	class ControlledRobot;


	class ControlledProcessManager
	{
	public:
		ControlledProcessManager(std::string processManagerName, const supplementary::AgentID* processManagerId, QBoxLayout* pmHorizontalLayout);
		virtual ~ControlledProcessManager();

		void updateGUI(std::chrono::system_clock::time_point now);
		void handleProcessStats(pair<std::chrono::system_clock::time_point, process_manager::ProcessStatsConstPtr> timePstsPair);
		void addRobot(QFrame* robot);
		void removeRobot(QFrame* robot);

		void hide();
		void show();

		std::chrono::duration<double> msgTimeOut;
		std::chrono::system_clock::time_point timeLastMsgReceived; /* < Time point, when the last message have been received */
		std::string name; /* < Hostname under which this process manager is running */
		const supplementary::AgentID* id; /* < The id of the host */
		supplementary::RobotExecutableRegistry* pmRegistry;

	private:
		std::map<const supplementary::AgentID*, ControlledRobot*, supplementary::AgentIDComparator> controlledRobotsMap; /* < The robots, which are controlled by this process manager */
		QBoxLayout* parentLayout;
		ControlledRobot* getControlledRobot(const supplementary::AgentID* robotId);
	};

} /* namespace pm_widget */
