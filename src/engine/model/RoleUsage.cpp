/*
 * RoleUsage.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: stefan
 */

#include "engine/model/RoleUsage.h"

namespace alica
{
	RoleUsage::RoleUsage(int priorityOrder, Role* role)
	{
		this->priorityOrder = priorityOrder;
		this->role = role;
		this->bUsed = false;
	}


	RoleUsage::~RoleUsage()
	{
		// TODO Auto-generated destructor stub
	}

	bool RoleUsage::isUsed() const
	{
		return bUsed;
	}

	void RoleUsage::setUsed(bool used)
	{
		bUsed = used;
	}

	int RoleUsage::getPriorityOrder() const
	{
		return priorityOrder;
	}

	void RoleUsage::setPriorityOrder(int priorityOrder)
	{
		this->priorityOrder = priorityOrder;
	}

	Role* RoleUsage::getRole() const
	{
		return role;
	}

	void RoleUsage::setRole(Role* role)
	{
		this->role = role;
	}

} /* namespace alica */
