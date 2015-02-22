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
		void queue4update(string execName, int execid, long pid);
		void update();
		void startExecutable(string execName, int execid);
		void startExecutable(string execName, int execid, vector<char*>& params);
		void changeDesiredState(string execName, int execid, bool shouldRun);
		void report(process_manager::ProcessStats& psts);
	private:
		map<int, ManagedExecutable*> executableMap;
	};

} /* namespace supplementary */

#endif /* MANAGEDROBOT_H_ */
