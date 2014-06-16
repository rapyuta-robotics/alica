/*
 * RobotEngineData.h
 *
 *  Created on: Jun 13, 2014
 *      Author: stefan
 */

#ifndef ROBOTENGINEDATA_H_
#define ROBOTENGINEDATA_H_

namespace alica
{
	class RobotProperties;
	class SuccessMarks;

	class RobotEngineData
	{
	public:
		RobotEngineData();
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

	protected:
		RobotProperties* properties;
		bool active;
		SuccessMarks* successMarks;
		unsigned long lastMessageTime;
	};

} /* namespace alica */

#endif /* ROBOTENGINEDATA_H_ */
