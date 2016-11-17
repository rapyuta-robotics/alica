/*
 * RoleAssignment.h
 *
 *  Created on: Jul 1, 2014
 *      Author: Stefan Jakob
 */

#ifndef ROLEASSIGNMENT_H_
#define ROLEASSIGNMENT_H_


#include <map>
#include <vector>
#include <list>
#include <memory>

#include "engine/IRoleAssignment.h"

using namespace std;
namespace alica
{

	class RoleSet;
	class Role;
	class RobotProperties;
	class ITeamObserver;
	class Publisher;
	class RobotRoleUtility;
	class RolePriority;
	class IAlicaCommunication;
	class AlicaEngine;

	class RoleAssignment : virtual public IRoleAssignment
	{
	public:
		RoleAssignment(AlicaEngine* ae);
		virtual ~RoleAssignment();
		void init();
		void tick();
		void setOwnRole(Role* ownRole);
		map<int, Role*>& getRobotRoleMapping();
		void update();

	private:
		AlicaEngine* ae;
		RoleSet* roleSet;
		map<int, Role*> robotRoleMapping;
		vector<RobotRoleUtility*> sortedRobots;/*<This vector is sorted each time an element is inserted */
		map<long, Role*> roles;
		unique_ptr<list<shared_ptr<RobotProperties>> > availableRobots;
		/**
		 * Current Robot's Properties.
		 */
		shared_ptr<RobotProperties> ownRobotProperties;
		void mapRoleToRobot(RolePriority* rp);

	protected:
		bool updateRoles = false;
		void roleUtilities();



	};

} /* namespace alica */

#endif /* ROLEASSIGNMENT_H_ */
