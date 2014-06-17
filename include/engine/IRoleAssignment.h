/*
 * IRoleAssignment.h
 *
 *  Created on: Jun 17, 2014
 *      Author: snook
 */

#ifndef IROLEASSIGNMENT_H_
#define IROLEASSIGNMENT_H_

using namespace std;

#include "model/Role.h"
#include <map>

namespace alica
{

	class IRoleAssignment
	{
	public:
		virtual ~IRoleAssignment() {}
		virtual void init() = 0;
		virtual void tick() = 0;
		virtual Role* getRole(int robotID) = 0;
		const Role* getOwnRole()
		{
			return ownRole;
		}
		map<int, Role*> getRobotRoleMapping()
		{
			return robotRoleMapping;
		}

	private:
		const Role* ownRole;
		map<int, Role*> robotRoleMapping;

	};
}
#endif /* IROLEASSIGNMENT_H_ */
