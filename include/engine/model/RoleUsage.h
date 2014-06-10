/*
 * RoleUsage.h
 *
 *  Created on: Jun 10, 2014
 *      Author: stefan
 */

#ifndef ROLEUSAGE_H_
#define ROLEUSAGE_H_

using namespace std;

#include "Role.h"

namespace alica
{

	class RoleUsage
	{
	public:
		RoleUsage(int priorityOrder, Role* role);
		virtual ~RoleUsage();
		bool isUsed() const;
		void setUsed(bool used);
		int getPriorityOrder() const;
		void setPriorityOrder(int priorityOrder);
		const Role*& getRole() const;
		void setRole(const Role*& role);

	protected:
		int priorityOrder;
		Role* role;
		bool bUsed;
	};

} /* namespace alica */

#endif /* ROLEUSAGE_H_ */
