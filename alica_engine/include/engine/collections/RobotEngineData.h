/*
 * RobotEngineData.h
 *
 *  Created on: Jun 13, 2014
 *      Author: Stefan Jakob
 */

#ifndef ROBOTENGINEDATA_H_
#define ROBOTENGINEDATA_H_


#include <map>
#include <typeinfo>
#include <memory>

using namespace std;
namespace alica
{
	class RobotProperties;
	class SuccessMarks;
	class Variable;
	class Role;
	class AlicaEngine;

	/**
	 * Basic Runtime information relating to a robot within the team
	 */
	class RobotEngineData
	{
	public:
		RobotEngineData(AlicaEngine* ae, shared_ptr<RobotProperties> properties);
		virtual ~RobotEngineData();
		bool isActive();
		void setActive(bool active);
		shared_ptr<RobotProperties> getProperties() ;
		void setProperties(shared_ptr<RobotProperties> properties);
		shared_ptr<SuccessMarks> getSuccessMarks();
		void setSuccessMarks(shared_ptr<SuccessMarks> successMarks);
		unsigned long getLastMessageTime();
		void setLastMessageTime(unsigned long lastMessageTime);
		virtual void initDomainVariables();
		virtual Variable* getDomainVariable(string sort);
		Role* getLastRole();
		void setLastRole(Role* lastRole);

	protected:
		AlicaEngine* ae;
		/**
		 * The robot's RobotProperties
		 */
		shared_ptr<RobotProperties> properties;
		/**
		 * Whether or not the robot is considered active
		 */
		bool active;
		/**
		 * The SuccessMarks of the robot, indicating which EntryPoints it completed.
		 */
		shared_ptr<SuccessMarks> successMarks;
		/**
		 * The timestamp of the last message event from this robot
		 */
		unsigned long lastMessageTime;
		map<string, Variable*> domainVariables;
		long makeUniqueId(string s);
		Role* lastRole;
	};

} /* namespace alica */

#endif /* ROBOTENGINEDATA_H_ */
