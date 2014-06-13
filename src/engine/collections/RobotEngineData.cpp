/*
 * RobotEngineData.cpp
 *
 *  Created on: Jun 13, 2014
 *      Author: stefan
 */

#include <engine/collections/RobotEngineData.h>

namespace alica
{

	RobotEngineData::RobotEngineData()
	{
		// TODO Auto-generated constructor stub

	}

	RobotEngineData::RobotEngineData(RobotProperties* properties)
	{
		this->properties = properties;
	}

	RobotEngineData::~RobotEngineData()
	{
		// TODO Auto-generated destructor stub
	}

	bool RobotEngineData::isActive() const
	{
		return active;
	}

	void RobotEngineData::setActive(bool active)
	{
		this->active = active;
	}

	RobotProperties* RobotEngineData::getProperties() const
	{
		return properties;
	}

	void RobotEngineData::setProperties(RobotProperties* properties)
	{
		this->properties = properties;
	}

} /* namespace alica */


