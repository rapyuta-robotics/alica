#pragma once

#include <engine/IRoleAssignment.h>
#include <memory>
#include <list>

namespace alica
{
	class AlicaEngine;
	class TeamObserver;
	class RobotProperties;

	class StaticRoleAssignment : public IRoleAssignment
	{
	public:
		StaticRoleAssignment(AlicaEngine* ae);
		~StaticRoleAssignment() = default;

		void init();
		void tick();
		Role* getRole(int robotId);
		void setCommunication(IAlicaCommunication* communication);
		void update();

		/**
		 * Calculates the actual role assignment and is triggered if an
		 * update is deemed necessary.
		 */
		void calculateRoles();

	private:
		bool updateRoles;

		AlicaEngine* ae;
		std::map<long, Role*> roles;
		std::unique_ptr<std::list<const RobotProperties*> > agentProperties;
	};

} /* namespace alica */
