/*
 * Role.h
 *
 *  Created on: Mar 5, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLE_H_
#define ROLE_H_

using namespace std;

#include <map>
#include <string>
#include <sstream>
#include <exception>

#include "AlicaElement.h"
#include "RoleTaskMapping.h"
#include "Characteristic.h"
#include "RoleDefinitionSet.h"

namespace alica
{
	class RoleTaskMapping;
	class RoleDefinitionSet;
	class Role : public AlicaElement
	{
	public:
		Role();
		virtual ~Role();

		double getPriority(long taskId);
		string toString();

		const map<string, Characteristic*>& getCharacteristics() const;
		const RoleDefinitionSet* getRoleDefinitionSet() const;
		void setRoleDefinitionSet(const RoleDefinitionSet* roleDefinitionSet);
		const RoleTaskMapping* getRoleTaskMapping() const;
		void setRoleTaskMapping(const RoleTaskMapping* roleTaskMapping);

	protected:
		const RoleTaskMapping* roleTaskMapping;
		map<string, Characteristic*> characteristics;
		const RoleDefinitionSet* roleDefinitionSet;
	};

} /* namespace Alica */

#endif /* ROLE_H_ */
