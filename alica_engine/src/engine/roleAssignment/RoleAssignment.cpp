/*
 * RoleAssignment.cpp
 *
 *  Created on: Jul 1, 2014
 *      Author: Stefan Jakob
 */

#include <engine/roleAssignment/RoleAssignment.h>
#include "engine/AlicaEngine.h"
#include "engine/model/Role.h"
#include "engine/ITeamObserver.h"

namespace alica
{

	RoleAssignment::RoleAssignment()
	{
		this->robotRoleMapping = map<int, Role*>();
		this->availableRobots = list<RobotProperties*>();
		this->sortedRobots = vector<RobotRoleUtility*>();
		//TODO
	}

	RoleAssignment::~RoleAssignment()
	{
	}

	Role* RoleAssignment::getOwnRole()
	{
		return ownRole;
	}

	void RoleAssignment::setOwnRole(Role* ownRole)
	{
		this->ownRole = ownRole;
	}

	void RoleAssignment::init()
	{
		this->to = AlicaEngine::getInstance()->getTeamObserver();
		//TODO delegates missing
		//to->onTeamCHangedEvent += Update;

		this->ownRobotProperties = to->getOwnRobotProperties();
		roleUtilities();
	}

	Role* RoleAssignment::getRole(int robotId)
	{
		Role* r = nullptr;
		auto iter = this->robotRoleMapping.begin();
		if(iter != this->robotRoleMapping.end())
		{
			return iter->second;

		}
	}

	void RoleAssignment::tick()
	{
	}

	map<int, Role*> RoleAssignment::getRobotRoleMapping()
	{
		return robotRoleMapping;
	}

	void RoleAssignment::mapRoleToRobot(RolePriority* rp)
	{
	}

	void RoleAssignment::roleUtilities()
	{
	}

	void RoleAssignment::update()
	{
		this->updateRoles = true;
	}

} /* namespace alica */
