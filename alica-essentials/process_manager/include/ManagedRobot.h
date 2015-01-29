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

using namespace std;

namespace supplementary
{
	class ManagedExecutable;

	class ManagedRobot
	{
	public:
		ManagedRobot(string robotName);
		virtual ~ManagedRobot();
		void queue4update(string execName, int execid, long pid);
		void update();
		void startExecutable(string execName, int execid);
		void startExecutable(string execName, int execid, char** params);
		void changeDesiredState(int execid, bool shouldRun);
	private:
		string robotName;
		map<int, ManagedExecutable*> executableMap;
	};

} /* namespace supplementary */

#endif /* MANAGEDROBOT_H_ */
