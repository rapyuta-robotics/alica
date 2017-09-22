#pragma once

#include "engine/IRobotID.h"

#include "model/Role.h"
#include "ITeamObserver.h"
#include "engine/collections/RobotEngineData.h"

#include <map>

namespace alica
{

	class AlicaEngine;
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

		const Role* getOwnRole();
		const Role* getRole(alica::IRobotID robotId);
		void setCommunication(IAlicaCommunication* communication);

	protected:
		AlicaEngine * engine;
		/**
		 * Current Robot's role.
		 */
		Role* ownRole;
		map<alica::IRobotID, Role*> robotRoleMapping;
		IAlicaCommunication* communication;

	};
}
