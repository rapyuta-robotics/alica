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

	RoleAssignment::RoleAssignment(AlicaEngine* ae) : ae(ae)
	{
		this->communication = nullptr;
		this->to = nullptr;
		this->ownRobotProperties = nullptr;
		this->roleSet = nullptr;
		this->rolePub = nullptr;
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

	Role* RoleAssignment::getRole(int robotId)
	{
		Role* r = nullptr;
		auto iter = this->robotRoleMapping.find(robotId);
		if (iter != this->robotRoleMapping.end())
		{
			return iter->second;
		}
		else
		{
			r = to->getRobotById(robotId)->getLastRole();
			if (r != nullptr)
			{
				return r;
			}
			cerr << "RA: There is no role assigned for robot: " << robotId << endl;
			throw new exception();
		}
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

		for (RoleUsage* rolUse : rp->getPriorityList())
		{

			for (RobotRoleUtility* rc : this->sortedRobots)
			{

				if (rolUse->getRole() == rc->getRole())
				{
					if (this->robotRoleMapping.size() != 0 && (this->robotRoleMapping.find(rc->getRobot()->getId()) != this->robotRoleMapping.end()
							|| rc->getUtilityValue() == 0))
					{
						continue;
					}
					this->robotRoleMapping.insert(pair<int, Role*>(rc->getRobot()->getId(), rc->getRole()));

					if (this->ownRobotProperties->getId() == rc->getRobot()->getId())
					{
						this->ownRole = rc->getRole();
					}

					to->getRobotById(rc->getRobot()->getId())->setLastRole(rc->getRole());

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
		for (auto roleIter = this->roles.begin(); roleIter != this->roles.end(); roleIter++)
		{
			for (auto rps : (*this->availableRobots))
			{
				int y = 0;
				dutility = 0;
				for (auto charIter = roleIter->second->getCharacteristics().begin();
						charIter != roleIter->second->getCharacteristics().end(); charIter++)
				{
					Characteristic* rbChar = nullptr;
					string s = charIter->second->getName();
//					auto iter = rps->getCharacteristics().find(charIter->second->getName());
					for (auto iter : rps->getCharacteristics())
					{
						if (iter.first.compare(s) == 0)
						{
							rbChar = iter.second;
							break;
						}
					}
					if (rbChar != nullptr)
					{
						double individualUtility = charIter->second->getCapability()->similarityValue(
								charIter->second->getCapValue(), rbChar->getCapValue());
						if (individualUtility == 0)
						{
							dutility = 0;
							break;
						}
						dutility += (charIter->second->getWeight() * individualUtility);
						y++;
					}
				}
				if (y != 0)
				{

					dutility /= y;
					rc = new RobotRoleUtility(dutility, rps, roleIter->second);
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
//			cout << "RA: manage role assignement." << endl;
			mapRoleToRobot(rp);
		}
		delete rp;
	}

	void RoleAssignment::setCommunication(IAlicaCommunication* communication)
	{
		this->communication = communication;
	}

	void RoleAssignment::update()
	{
		this->updateRoles = true;
	}

} /* namespace alica */
