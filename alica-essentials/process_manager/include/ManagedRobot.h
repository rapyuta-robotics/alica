/*
 * ManagedRobot.h
 *
 *  Created on: Nov 28, 2014
 *      Author: Stephan Opfer
 */

#ifndef MANAGEDROBOT_H_
#define MANAGEDROBOT_H_

#define MNG_ROBOT_DEBUG

#include <map>
#include <string>
#include <vector>
#include "process_manager/ProcessStat.h"
#include "process_manager/ProcessStats.h"
#include "ProcessManagerRegistry.h"
#include "RobotMetaData.h"


using namespace std;

namespace supplementary
{
	class ManagedExecutable;

	class ManagedRobot : public RobotMetaData
	{
	public:
		ManagedRobot(string robotName, int id);
		virtual ~ManagedRobot();
		void queue4update(int execid, long pid, RobotExecutableRegistry* registry);
		void update(unsigned long long cpuDelta);
		void startExecutable(string execName, int execid);
		void startExecutable(string execName, int execid, vector<char*>& params);
		void changeDesiredState(int execId, bool shouldRun, RobotExecutableRegistry* registry);
		void report(process_manager::ProcessStats& psts);
	private:
		map<int, ManagedExecutable*> executableMap;
	};

} /* namespace supplementary */

#endif /* MANAGEDROBOT_H_ */
