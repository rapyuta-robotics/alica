/*
 * IRoleAssignment1.cpp
 *
 *  Created on: 17 Nov 2016
 *      Author: Stephan Opfer
 */

#include "engine/IRoleAssignment.h"

namespace alica
{
	IRoleAssignment::IRoleAssignment(AlicaEngine* ae) :
			ownRole(nullptr), robotRoleMapping(map<int, Role*>()), to(nullptr), communication(nullptr)
	{
	}

	Role* IRoleAssignment::getOwnRole()
	{
		return ownRole;
	}

	Role* IRoleAssignment::getRole(int robotId)
	{
		Role* r = nullptr;
		auto iter = this->robotRoleMapping.find(robotId);
		if (iter != this->robotRoleMapping.end())
		{
			return iter->second;
		}
		else
		{
			r = this->to->getRobotById(robotId)->getLastRole();
			if (r != nullptr)
			{
				return r;
			}
			cerr << "RA: There is no role assigned for robot: " << robotId << endl;
			throw new exception();
		}
	}

	void IRoleAssignment::setCommunication(IAlicaCommunication* communication)
	{
		this->communication = communication;
	}

} /* namespace alica */
