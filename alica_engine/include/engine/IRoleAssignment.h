/*
 * IRoleAssignment.h
 *
 *  Created on: Jun 17, 2014
 *      Author: Paul Panin
 */

#ifndef IROLEASSIGNMENT_H_
#define IROLEASSIGNMENT_H_

using namespace std;

#include "engine/IRobotID.h"

#include "model/Role.h"
#include "ITeamObserver.h"
#include "engine/collections/RobotEngineData.h"
#include <map>

namespace alica
{
	class IAlicaCommunication;

	class IRoleAssignment
	{
	public:

		IRoleAssignment(AlicaEngine* ae);
		virtual ~IRoleAssignment()
		{
		}

		virtual void init() = 0;
		virtual void tick() = 0;
		virtual void update() = 0;

		Role* getOwnRole();
		Role* getRole(alica::IRobotID robotId);
		void setCommunication(IAlicaCommunication* communication);

	protected:
		/**
		 * Current Robot's role.
		 */
		Role* ownRole;
		map<alica::IRobotID, Role*> robotRoleMapping;
		IAlicaCommunication* communication;
		ITeamObserver* to;

	};
}
#endif /* IROLEASSIGNMENT_H_ */
