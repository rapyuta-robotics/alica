/*
 * StaticRoleAssignment.h
 *
 *  Created on: 17 Nov 2016
 *      Author: Stephan Opfer
 */

#ifndef SRC_ENGINE_STATICROLEASSIGNMENT_STATICROLEASSIGNMENT_H_
#define SRC_ENGINE_STATICROLEASSIGNMENT_STATICROLEASSIGNMENT_H_

#include <engine/IRoleAssignment.h>

namespace alica
{
	class AlicaEngine;
	class TeamObserver;
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
		map<long, Role*> roles;
		unique_ptr<list<shared_ptr<RobotProperties>> > availableRobots;
	};

} /* namespace alica */

#endif /* SRC_ENGINE_STATICROLEASSIGNMENT_STATICROLEASSIGNMENT_H_ */
