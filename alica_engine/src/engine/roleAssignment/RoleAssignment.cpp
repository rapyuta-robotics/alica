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
#include "engine/collections/RobotEngineData.h"
#include "engine/model/RoleUsage.h"
#include "engine/PlanRepository.h"
#include "engine/roleAssignment/RobotRoleUtility.h"
#include "engine/collections/RobotProperties.h"
#include "engine/model/RolePriority.h"
#include "engine/model/Characteristic.h"
#include "engine/model/Capability.h"
#include "engine/containers/RoleSwitch.h"
#include "engine/IAlicaCommunication.h"

namespace alica
{

	RoleAssignment::RoleAssignment()
	{
		this->communication = nullptr;
		this->to = nullptr;
		this->ownRobotProperties = nullptr;
		this->roleSet = nullptr;
		this->rolePub = nullptr;
		this->ownRole = nullptr;
		this->robotRoleMapping = map<int, Role*>();
		this->availableRobots;
		this->sortedRobots = vector<RobotRoleUtility*>();
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
		if (this->ownRole != ownRole)
		{
			RoleSwitch rs = RoleSwitch();
			rs.roleID = ownRole->getId();
			this->communication->SendRoleSwitch(rs);
		}
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

	map<int, Role*> RoleAssignment::getRobotRoleMapping()
	{
		return robotRoleMapping;
	}

	void RoleAssignment::mapRoleToRobot(RolePriority* rp)
	{
		cout << "MAP ROLe" << endl;
		for (RoleUsage* rolUse : rp->getPriorityList())
		{
			for (RobotRoleUtility* rc : this->sortedRobots)
			{
				if (rolUse->getRole() == rc->getRole())
				{
					if (this->robotRoleMapping.find(rc->getRobot()->getId()) != this->robotRoleMapping.end()
							|| rc->getUtilityValue() == 0)
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

	void RoleAssignment::roleUtilities()
	{
		this->roleSet = AlicaEngine::getInstance()->getRoleSet();
		this->roles = AlicaEngine::getInstance()->getPlanRepository()->getRoles();
		cout << "ROLE UTILITIES" << endl;
		if (this->roleSet == nullptr)
		{
			cerr << "RA: The current Roleset is null!" << endl;
			throw new exception();
		}
		this->availableRobots = AlicaEngine::getInstance()->getTeamObserver()->getAvailableRobotProperties();

		cout << "RA: Available robots: " << this->availableRobots->size() << endl;
		for (RobotProperties* aRobot : (*this->availableRobots))
		{
			cout << aRobot->getId() << " " << endl;
		}
		cout << endl;
		double dutility = 0;
		RobotRoleUtility* rc;
		this->sortedRobots.clear();

		for(auto roleIter = this->roles.begin(); roleIter != this->roles.end(); roleIter++)
		{
			for(RobotProperties* rps : (*this->availableRobots))
			{
				int y = 0;
				dutility = 0;
				for(auto charIter = roleIter->second->getCharacteristics().begin(); charIter != roleIter->second->getCharacteristics().end(); charIter++)
				{
					cout << "CHARA" << endl;
					Characteristic* rbChar = nullptr;
					for(auto iter : rps->getCharacteristics())
					{
						cout << "CNAME " << iter.second->getCapability()->getName() << endl;
					}
					for(auto iter : roleIter->second->getCharacteristics())
					{
						cout << "DNAME " << iter.second->getName() << endl;
					}
					for(auto iter : rps->getCharacteristics())
					{
						cout << "STRING " << iter.first << endl;
					}
					string s = charIter->second->getName();
					cout << "Ausgabe " << s << endl;
					auto iter = rps->getCharacteristics().find(charIter->second->getName());
					cout << "ITER AUSGABE " << iter->second->getName() << endl;
					if(iter != rps->getCharacteristics().end())
					{
						cout << "WAR ICH NIE DRINNE " << endl;
						rbChar = iter->second;
						cout << "NAME " << rbChar->getName() << endl;
						double individualUtility = charIter->second->getCapability()->similarityValue(charIter->second->getCapValue(), rbChar->getCapValue());
						if(individualUtility == 0)
						{
							dutility = 0;
							break;
						}
						dutility += (charIter->second->getWeight() * individualUtility);
						y++;
					}
				}
				if(y != 0)
				{
					dutility /= y;
					rc = new RobotRoleUtility(dutility, rps, roleIter->second);
					cout << "PUSHE" << rc->getRobot()->getName() << endl;
					this->sortedRobots.push_back(rc);
					sort(this->sortedRobots.begin(), this->sortedRobots.end(), RobotRoleUtility::compareTo);
				}
			}
		}
		cout << this->sortedRobots.size() << endl;
		if (this->sortedRobots.size() == 0)
		{
			AlicaEngine::getInstance()->abort(
					"RA: Could not establish a mapping between robots and roles. Please check capability definitions!");
		}
		cout << "KOMME ICH RAUS" << endl;
		RolePriority* rp = new RolePriority();
		cout << "KOMME ICH RAUS" << endl;
		this->robotRoleMapping.clear();
		cout << "KOMME ICH RAUS" << endl;
		while (this->robotRoleMapping.size() < this->availableRobots->size())
		{
			mapRoleToRobot(rp);
		}
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
