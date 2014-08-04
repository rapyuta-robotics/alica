/*
 * RobotEngineData.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#ifndef ROBOTENGINEDATA_H_
#define ROBOTENGINEDATA_H_

using namespace std;

#include <map>
#include <typeinfo>

namespace alica
{
	class RobotProperties;
	class SuccessMarks;
	class Variable;
	class Role;

	class RobotEngineData
	{
	public:
		RobotEngineData(RobotProperties* properties);
		virtual ~RobotEngineData();
		bool isActive() const;
		void setActive(bool active);
		RobotProperties* getProperties() const;
		void setProperties(RobotProperties* properties);
		SuccessMarks* getSuccessMarks() const;
		void setSuccessMarks(SuccessMarks* successMarks);
		unsigned long getLastMessageTime() const;
		void setLastMessageTime(unsigned long lastMessageTime);
		virtual void initSortedTerms();
		virtual Variable* getSortedVariable(string sort);
		Role* getLastRole();
		void setLastRole(Role* lastRole);

	protected:
		RobotProperties* properties;
		bool active;
		SuccessMarks* successMarks;
		unsigned long lastMessageTime;
		map<string, Variable*> sortedVariables;
		long makeUniqueId(string s);
		Role* lastRole;
	};

} /* namespace alica */

#endif /* ROBOTENGINEDATA_H_ */
