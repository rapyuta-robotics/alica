/*
 * StaticRoleAssignment.cpp
 *
 *  Created on: 17 Nov 2016
 *      Author: Stephan Opfer
 */

#include "engine/staticroleassignment/StaticRoleAssignment.h"
#include "engine/collections/RobotProperties.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/containers/RoleSwitch.h"
#include "engine/IAlicaCommunication.h"

namespace alica
{

	StaticRoleAssignment::StaticRoleAssignment(AlicaEngine* ae) : IRoleAssignment(ae), ae(ae), updateRoles(false)
	{
	}

	StaticRoleAssignment::~StaticRoleAssignment()
	{
	}

	void StaticRoleAssignment::init()
	{
		this->to = ae->getTeamObserver();

		this->calculateRoles();
	}

	void StaticRoleAssignment::tick()
	{
		if (this->updateRoles)
		{
			this->updateRoles = false;
			this->calculateRoles();
		}
	}

	void StaticRoleAssignment::update()
	{
		this->updateRoles = true;
	}

	void StaticRoleAssignment::calculateRoles()
	{
		// clear current map
		this->robotRoleMapping.clear();

		// get data for "calculations"
		this->roles = ae->getPlanRepository()->getRoles();
		this->availableRobots = ae->getTeamObserver()->getAvailableRobotProperties();

		// assign a role for each robot if you have match
		for (auto& robot : *availableRobots)
		{
			bool roleIsAssigned = false;

			for (auto& role : roles)
			{
				// make entry in the map if the roles match
				if (role.second->getName() == robot->getDefaultRole())
				{
					this->robotRoleMapping.emplace(robot->getId(), role.second);

					// set own role, if its me
					if (robot->getId() == this->to->getOwnId())
					{
						if (this->ownRole != role.second)
						{
							this->ownRole = role.second;

							// probably nothing is reacting on this message, but anyway we send it
							if (this->communication != nullptr)
							{
								RoleSwitch rs;
								rs.roleID = role.first;
								this->communication->sendRoleSwitch(rs);
							}
						}
					}
					roleIsAssigned = true;
					break;
				}
			}

			if (!roleIsAssigned)
			{
				ae->abort("RA: Could not set a role for robot " + robot->getName() + " with default role " + robot->getDefaultRole() + "!");
			}
		}
	}

} /* namespace alica */
