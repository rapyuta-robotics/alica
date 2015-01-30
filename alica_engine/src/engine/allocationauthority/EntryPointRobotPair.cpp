/*
 * EntryPointRobotPair.cpp
 *
 *  Created on: Jul 17, 2014
 *      Author: Stefan Jakob
 */

#include <engine/allocationauthority/EntryPointRobotPair.h>

namespace alica
{
	EntryPointRobotPair::~EntryPointRobotPair()
	{
	}

	EntryPoint* EntryPointRobotPair::getEntryPoint()
	{
		return entryPoint;
	}

	void EntryPointRobotPair::setEntryPoint(EntryPoint* entryPoint)
	{
		this->entryPoint = entryPoint;
	}

	int EntryPointRobotPair::getRobot()
	{
		return robot;
	}

	EntryPointRobotPair::EntryPointRobotPair(EntryPoint* ep, int r)
	{
		this->entryPoint = ep;
		this->robot = r;
	}

	void EntryPointRobotPair::setRobot(int robot)
	{
		this->robot = robot;
	}

	bool EntryPointRobotPair::equals(std::shared_ptr<EntryPointRobotPair> thisOne, std::shared_ptr<EntryPointRobotPair> other)
	{
		if (other == nullptr)
		{
			return false;
		}
		if (other->entryPoint->getId() != thisOne->entryPoint->getId())
			return false;
		return (other->getRobot() == thisOne->robot);
	}

} /* namespace alica */
