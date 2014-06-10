/*
 * RoleTaskMapping.h
 *
 *  Created on: Mar 8, 2014
 *      Author: Stephan Opfer
 */

#ifndef ROLETASKMAPPING_H_
#define ROLETASKMAPPING_H_

using namespace std;

#include <map>
#include <string>
#include <sstream>

#include "engine/model/AlicaElement.h"
#include "Role.h"

namespace alica
{

	/*
	 *
	 */
	class Role;
	class RoleTaskMapping : public AlicaElement
	{
	public:
		RoleTaskMapping();
		virtual ~RoleTaskMapping();

		string toString();

		const Role* getRole() const;
		void setRole(const Role* role);
		const map<long, double>& getTaskPriorities() const;
		void setTaskPriorities(const map<long, double>& taskPriorities);

	protected:
		const Role* role;
		map<long,double> taskPriorities;
	};

} /* namespace Alica */

#endif /* ROLETASKMAPPING_H_ */
