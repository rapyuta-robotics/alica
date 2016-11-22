/*
 * RoleAssignment.cpp
 *
 *  Created on: Jul 1, 2014
 *      Author: Stefan Jakob
 */

#include <engine/roleassignment/RoleAssignment.h>
#include "engine/AlicaEngine.h"
#include "engine/model/Role.h"
#include "engine/ITeamObserver.h"
#include "engine/collections/RobotEngineData.h"
#include "engine/model/RoleUsage.h"
#include "engine/PlanRepository.h"
#include "engine/roleassignment/RobotRoleUtility.h"
#include "engine/collections/RobotProperties.h"
#include "engine/model/RolePriority.h"
#include "engine/model/Characteristic.h"
#include "engine/model/Capability.h"
#include "engine/containers/RoleSwitch.h"
#include "engine/IAlicaCommunication.h"

namespace alica
{

	RoleAssignment::RoleAssignment(AlicaEngine* ae) : IRoleAssignment(ae), ae(ae)
	{
		this->ownRobotProperties = nullptr;
		this->roleSet = nullptr;
		this->ownRole = nullptr;
		this->robotRoleMapping = map<int, Role*>();
		this->availableRobots = nullptr;
		this->sortedRobots = vector<RobotRoleUtility*>();
	}

	RoleAssignment::~RoleAssignment()
	{
		for(auto i : this->sortedRobots)
		{
			delete i;
		}
	}

	void RoleAssignment::setOwnRole(Role* ownRole)
	{
		if (this->ownRole != ownRole)
		{
			RoleSwitch rs = RoleSwitch();
			rs.roleID = ownRole->getId();
			this->communication->sendRoleSwitch(rs);
		}
		this->ownRole = ownRole;
	}

	void RoleAssignment::init()
	{
		this->to = ae->getTeamObserver();
		//TODO delegates missing
		//to->onTeamCHangedEvent += Update;

		this->ownRobotProperties = to->getOwnRobotProperties();
		roleUtilities();
	}

	void RoleAssignment::tick()
	{
		if (this->updateRoles)
		{
			this->updateRoles = false;
			this->roleUtilities();
		}
	}

	map<int, Role*>& RoleAssignment::getRobotRoleMapping()
	{
		return robotRoleMapping;
	}

	/**
	 * Maps the role to robot according to priority list and utility value.
	 * @param rp A RolePriority*
	 */
	void RoleAssignment::mapRoleToRobot(RolePriority* rp)
	{
		std::sort(this->sortedRobots.begin(), this->sortedRobots.end(), &RobotRoleUtility::compareTo);

		for (RoleUsage* roleUsage : rp->getPriorityList())
		{

			for (RobotRoleUtility* robRoleUtil : this->sortedRobots)
			{

				if (roleUsage->getRole() == robRoleUtil->getRole())
				{
					if (this->robotRoleMapping.size() != 0 && (this->robotRoleMapping.find(robRoleUtil->getRobot()->getId()) != this->robotRoleMapping.end()
							|| robRoleUtil->getUtilityValue() == 0))
					{
						continue;
					}
					this->robotRoleMapping.insert(pair<int, Role*>(robRoleUtil->getRobot()->getId(), robRoleUtil->getRole()));

					if (this->ownRobotProperties->getId() == robRoleUtil->getRobot()->getId())
					{
						this->ownRole = robRoleUtil->getRole();
					}

					to->getRobotById(robRoleUtil->getRobot()->getId())->setLastRole(robRoleUtil->getRole());

					break;
				}
			}
		}
	}

	/**
	 * Assign roles according to capability and role priority list
	 * @exception Represents errors that occur during application execution.
	 */
	void RoleAssignment::roleUtilities()
	{
		this->roleSet = ae->getRoleSet();
		this->roles = ae->getPlanRepository()->getRoles();
		if (this->roleSet == nullptr)
		{
			cerr << "RA: The current Roleset is null!" << endl;
			throw new exception();
		}
		this->availableRobots = ae->getTeamObserver()->getAvailableRobotProperties();

		cout << "RA: Available robots: " << this->availableRobots->size() << endl;
		cout << "RA: Robot Ids: ";
		for (auto aRobot : (*this->availableRobots))
		{
			cout << aRobot->getId() << " ";
		}
		cout << endl;
		double dutility = 0;
		RobotRoleUtility* rc;
		this->sortedRobots.clear();
		for (auto& roleEntry : this->roles)
		{
			for (auto robProperties : (*this->availableRobots))
			{
				int y = 0;
				dutility = 0;
				for (auto& roleCharacEntry : roleEntry.second->getCharacteristics())
				{
					// find the characteristics object of a robot
					Characteristic* rbChar = nullptr;
					string roleCharacName = roleCharacEntry.second->getName();
					for (auto& robotCharac : robProperties->getCharacteristics())
					{
						if (robotCharac.first.compare(roleCharacName) == 0)
						{
							rbChar = robotCharac.second;
							break;
						}
					}

					if (rbChar != nullptr)
					{
						double individualUtility = roleCharacEntry.second->getCapability()->similarityValue(
								roleCharacEntry.second->getCapValue(), rbChar->getCapValue());
						if (individualUtility == 0)
						{
							dutility = 0;
							break;
						}
						dutility += (roleCharacEntry.second->getWeight() * individualUtility);
						y++;
					}
				}
				if (y != 0)
				{
					dutility /= y;
					rc = new RobotRoleUtility(dutility, robProperties, roleEntry.second);
					this->sortedRobots.push_back(rc);
					sort(this->sortedRobots.begin(), this->sortedRobots.end(), RobotRoleUtility::compareTo);
				}
			}
		}
		if (this->sortedRobots.size() == 0)
		{
			ae->abort("RA: Could not establish a mapping between robots and roles. Please check capability definitions!");
		}
		RolePriority* rp = new RolePriority(ae);
		this->robotRoleMapping.clear();
		while (this->robotRoleMapping.size() < this->availableRobots->size())
		{
			mapRoleToRobot(rp);
		}
		delete rp;
	}



	void RoleAssignment::update()
	{
		this->updateRoles = true;
	}

} /* namespace alica */
