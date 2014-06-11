/*
 * RoleSet.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLESET_H_
#define ROLESET_H_

using namespace std;

#include <list>
#include <string>
#include <sstream>

#include "AlicaElement.h"

namespace alica
{
	class RoleTaskMapping;

	class RoleSet : public AlicaElement
	{
	public:
		RoleSet();
		virtual ~RoleSet();
		string toString();
		bool isIsDefault() const;
		void setIsDefault(bool isDefault);
		const list<RoleTaskMapping*>& getRoleTaskMappings() const;
		void setRoleTaskMappings(const list<RoleTaskMapping*>& roleTaskMappings);
		long getUsableWithPlanId() const;
		void setUsableWithPlanId(long usableWithPlanId);

	protected:
		list<RoleTaskMapping*> roleTaskMappings;
		bool isDefault;
		long usableWithPlanID;
	};

} /* namespace Alica */

#endif /* ROLESET_H_ */
