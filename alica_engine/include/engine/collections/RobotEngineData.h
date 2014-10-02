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

	/**
	 * Basic Runtime information relating to a robot within the team
	 */
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
		/**
		 * The robot's RobotProperties
		 */
		RobotProperties* properties;
		/**
		 * Whether or not the robot is considered active
		 */
		bool active;
		/**
		 * The SuccessMarks of the robot, indicating which EntryPoints it completed.
		 */
		SuccessMarks* successMarks;
		/**
		 * The timestamp of the last message event from this robot
		 */
		unsigned long lastMessageTime;
		map<string, Variable*> sortedVariables;
		long makeUniqueId(string s);
		Role* lastRole;
	};

} /* namespace alica */

#endif /* ROBOTENGINEDATA_H_ */
