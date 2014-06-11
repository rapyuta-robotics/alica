/*
 * RoleDefinitionSet.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLEDEFINITIONSET_H_
#define ROLEDEFINITIONSET_H_

using namespace std;

#include <list>
#include <string>

#include "AlicaElement.h"

namespace alica
{
	class Role;

	class RoleDefinitionSet : public AlicaElement
	{
	public:
		RoleDefinitionSet();
		virtual ~RoleDefinitionSet();
		const string& getFileName() const;
		void setFileName(const string& fileName);
		const list<Role*>& getRoles() const;

	private:
		void setRoles(const list<Role*>& roles);

	protected:
		string fileName;
		list<Role*> roles;
	};

} /* namespace Alica */

#endif /* ROLEDEFINITIONSET_H_ */
