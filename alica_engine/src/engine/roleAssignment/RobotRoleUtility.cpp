/*
 * RobotRoleUtility.cpp
 *
 *  Created on: Jul 1, 2014
 *      Author: Stefan Jakob
 */

#include <engine/roleAssignment/RobotRoleUtility.h>
#include "engine/model/Role.h"
#include "engine/collections/RobotProperties.h"

namespace alica
{

	RobotRoleUtility::RobotRoleUtility(double dUtilityValue, RobotProperties* robot, Role* role)
	{
		this->utilityValue = dUtilityValue;
		this->robot = robot;
		this->role = role;
	}

	RobotRoleUtility::~RobotRoleUtility()
	{
		// TODO Auto-generated destructor stub
	}

	RobotProperties* RobotRoleUtility::getRobot()
	{
		return robot;
	}

	Role* RobotRoleUtility::getRole()
	{
		return role;
	}

	bool RobotRoleUtility::compareTo(RobotRoleUtility* thisOne, RobotRoleUtility* otherOne)
	{
		bool compare = (otherOne->getRole()->getId() == thisOne->getRole()->getId());
		if(compare)
		{
			compare = (otherOne->getUtilityValue() == thisOne->getUtilityValue());
		}
		if(compare)
		{
			compare = (otherOne->getRobot()->getId() == thisOne->getRobot()->getId());
		}
		return compare;
	}

	double RobotRoleUtility::getUtilityValue()
	{
		return utilityValue;
	}

} /* namespace alica */
