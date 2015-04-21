/*
 * RolePriority.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLEPRIORITY_H_
#define ROLEPRIORITY_H_


#include <map>
#include <list>
#include <string>
#include <SystemConfig.h>

#include "AlicaElement.h"

using namespace std;
namespace alica
{
	class Role;
	class RoleUsage;
	class AlicaEngine;

	class RolePriority : public AlicaElement
	{
	public:
		RolePriority(AlicaEngine* ae);
		virtual ~RolePriority();
		list<RoleUsage*>& getPriorityList();

	private:
		map<long, Role*> roles;
		Role* role;
		list<RoleUsage*> priorityList;

	protected:

	};

} /* namespace Alica */

#endif /* ROLEPRIORITY_H_ */
