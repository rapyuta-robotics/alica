/*
 * RoleDefinitionSet.cpp
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#include "engine/model/RoleDefinitionSet.h"

namespace alica
{

	RoleDefinitionSet::RoleDefinitionSet()
	{

	}

	RoleDefinitionSet::~RoleDefinitionSet()
	{
	}

	const string& RoleDefinitionSet::getFileName() const
	{
		if (this->getFileName().empty())
		{
			static string result = name + ".rdefset";
			return result;
		}
		return fileName;
	}

	void RoleDefinitionSet::setFileName(const string& fileName)
	{
		this->fileName = fileName;
	}

	const list<Role*>& RoleDefinitionSet::getRoles() const
	{
		return roles;
	}

	void RoleDefinitionSet::setRoles(const list<Role*>& roles)
	{
		this->roles = roles;
	}
} /* namespace Alica */


