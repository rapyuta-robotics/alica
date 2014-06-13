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

	protected:
		RobotProperties* properties;
		bool active;
	};

} /* namespace alica */

#endif /* ROBOTENGINEDATA_H_ */
