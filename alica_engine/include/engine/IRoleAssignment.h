/*
 * IRoleAssignment.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#ifndef IROLEASSIGNMENT_H_
#define IROLEASSIGNMENT_H_

using namespace std;

#include "model/Role.h"
#include <map>

namespace alica
{
	class IAlicaCommunication;
	class IRoleAssignment
	{
	public:

		virtual ~IRoleAssignment()
		{
		}
		virtual void init() = 0;
		virtual void tick() = 0;
		virtual Role* getRole(int robotID) = 0;
		virtual void setCommunication(IAlicaCommunication* communication) = 0;
		Role* getOwnRole()
		{
			return ownRole;
		}
		map<int, Role*>& getRobotRoleMapping()
		{
			return robotRoleMapping;
		}
		virtual void update() = 0;

	protected:
		/**
		 * Current Robot's role.
		 */
		Role* ownRole;
		map<int, Role*> robotRoleMapping;

	};
}
#endif /* IROLEASSIGNMENT_H_ */
