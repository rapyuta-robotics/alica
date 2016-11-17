/*
 * RoleUsage.cpp
 *
 *  Created on: Jun 10, 2014
 *      Author: Stefan Jakob
 */

#include "engine/model/RoleUsage.h"

namespace alica
{
	RoleUsage::RoleUsage(int priorityOrder, Role* role)
	{
		this->priorityOrder = priorityOrder;
		this->role = role;
		this->used = false;
	}

	RoleUsage::~RoleUsage()
	{
	}

	bool RoleUsage::isUsed() const
	{
		return this->used;
	}

	void RoleUsage::setUsed(bool used)
	{
		this->used = used;
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
