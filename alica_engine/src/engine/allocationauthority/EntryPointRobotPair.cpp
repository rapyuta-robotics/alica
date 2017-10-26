#include "engine/allocationauthority/EntryPointRobotPair.h"

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

	const supplementary::IAgentID* EntryPointRobotPair::getRobot()
	{
		return robot;
	}

	EntryPointRobotPair::EntryPointRobotPair(EntryPoint* ep, const supplementary::IAgentID* r)
	{
		this->entryPoint = ep;
		this->robot = r;
	}

	void EntryPointRobotPair::setRobot(const supplementary::IAgentID* robot)
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
		return (*(other->getRobot()) == *(thisOne->robot));
	}

} /* namespace alica */
