/*
 * RolePriority.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/RolePriority.h"
#include "engine/AlicaEngine.h"
#include "engine/PlanRepository.h"
#include "engine/model/Role.h"
#include "engine/model/RoleUsage.h"

namespace alica
{

	RolePriority::RolePriority()
	{
		supplementary::SystemConfig* sc = supplementary::SystemConfig::getInstance();
		this->roles = AlicaEngine::getInstance()->getPlanRepository()->getRoles();

		shared_ptr<vector<string> > priorities = (*sc)["Globals"]->getNames("Globals", "RolePriority", NULL);
		int order = 0;
		for (string iter : *priorities)
		{
			order = (*sc)["Globals"]->get<int>("Globals", "RolePriority", iter.c_str(), NULL);
			for (auto rolePair : this->roles)
			{
				if (rolePair.second->getName().compare(iter) == 0)
				{
					this->role = rolePair.second;
					break;
				}
			}
			this->priorityList.push_back(new RoleUsage(order, this->role));
		}
	}

	RolePriority::~RolePriority()
	{
	}

	list<RoleUsage*> RolePriority::getPriorityList()
	{
		return priorityList;
	}

} /* namespace Alica */
