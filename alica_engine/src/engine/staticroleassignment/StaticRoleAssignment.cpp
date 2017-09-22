/*
 * StaticRoleAssignment.cpp
 *
 *  Created on: 17 Nov 2016
 *      Author: Stephan Opfer
 */

//#define STATIC_RA_DEBUG

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

	/**
	 * Initially calculates the robot-role mapping once.
	 */
	void StaticRoleAssignment::init()
	{
		this->calculateRoles();
	}

	/**
	 * Triggers the recalculation of the robot-role mapping, if the updateRoles flag is set to true.
	 */
	void StaticRoleAssignment::tick()
	{
		if (this->updateRoles)
		{
			this->updateRoles = false;
			this->calculateRoles();
		}
	}

	/**
	 * Sets the updateRoles flag to true, in order to recalculate the robot-role mapping on the next tick.
	 */
	void StaticRoleAssignment::update()
	{
		this->updateRoles = true;
	}

	/**
	 * Recalculates the complete mapping from robot to role.
	 */
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
#ifdef STATIC_RA_DEBUG
					std::cout << "Static RA: Setting Role " << role.second->getName() << " for robot ID " << robot->getId() << std::endl;
#endif
					this->robotRoleMapping.emplace(robot->getId(), role.second);

					// set own role, if its me
					if (robot->getId() == this->ae->getTeamManager()->getOwnRobotID() && this->ownRole != role.second)
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
