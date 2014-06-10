/*
 * RolePriority.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/RolePriority.h"

namespace alica
{

	RolePriority::RolePriority()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->roles = AlicaEngine::getInstance()->getPlanRepository()->getRoles();
		// TODO
//		string[] priorities = (*sc)["Globals"].get<string[]>("Globals","RoleRipository");
		int order = 0;
//		for (int i = 0; i < priorities.size(); i++)
//		{
//			order = (*sc)["Globals"].get<int>("Globals", "RolePriority", priorities[i]);
//			for (Role* r : this->roles)
//			{
//				if (r->name.compare(priorities[i]))
//				{
//					this->role = r;
//					break;
//				}
//			}
//			this->priorityList.push_back(new RoleUsage(order, this->role)):
//		}
	}

	RolePriority::~RolePriority()
	{
	}

	const list<RoleUsage*>& RolePriority::getPriorityList() const
	{
		return priorityList;
	}

} /* namespace Alica */
