/*
 * RoleSet.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLESET_H_
#define ROLESET_H_


#include <list>
#include <string>
#include <sstream>

#include "AlicaElement.h"

using namespace std;
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
		list<RoleTaskMapping*>& getRoleTaskMappings();
		void setRoleTaskMappings(const list<RoleTaskMapping*> roleTaskMappings);
		long getUsableWithPlanId() const;
		void setUsableWithPlanId(long usableWithPlanId);

	protected:
		list<RoleTaskMapping*> roleTaskMappings;
		bool isDefault;
		/**
		 * the plan ID this roleset is defined for
		 */
		long usableWithPlanID;
	};

} /* namespace Alica */

#endif /* ROLESET_H_ */
