/*
 * RoleTaskMapping.cpp
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/RoleTaskMapping.h"

namespace alica
{

	RoleTaskMapping::RoleTaskMapping()
	{

	}

	RoleTaskMapping::~RoleTaskMapping()
	{
	}

	string RoleTaskMapping::toString()
	{
		stringstream ss;
		ss << "#RoleTaskMapping: " << this->name << " " << this->id << endl;
		ss << "\t Role-Name: " << this->role->getName() << endl;
		ss << "\t TaskPriorities: " << this->taskPriorities.size() << endl;
		for(map<long, double>::const_iterator iterator = this->taskPriorities.begin(); iterator != this->taskPriorities.end(); iterator++)
		{
			const long l = iterator->first;
			const double val = iterator->second;
			ss << "\t" << l << " : " << val << endl;
		}
		ss << endl;
		ss << "#EndRoleTaskMapping" << endl;
		return ss.str();
	}

//========================= Getter and Setter ======================

	const Role* RoleTaskMapping::getRole() const
	{
		return role;
	}

	void RoleTaskMapping::setRole(const Role* role)
	{
		this->role = role;
	}

	const map<long, double>& RoleTaskMapping::getTaskPriorities() const
	{
		return taskPriorities;
	}

	void RoleTaskMapping::setTaskPriorities(const map<long, double>& taskPriorities)
	{
		this->taskPriorities = taskPriorities;
	}

} /* namespace Alica */


