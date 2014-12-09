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
		ManagedRobot();
		virtual ~ManagedRobot();
		void queue4update(uint8_t execid, long pid);
		void queue4update(string execName, uint8_t execid, long pid);
		void update();

	private:
		map<uint8_t, ManagedExecutable*> executableMap;
	};

} /* namespace supplementary */

#endif /* MANAGEDROBOT_H_ */
